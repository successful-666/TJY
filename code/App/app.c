/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层：把各层服务按正确顺序装配起来
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app.h"

#include "main.h"
#include "log.h"
#include "app_eth.h"
#include "app_cmd.h"
#include "chassis.h"
#include "svc_can.h"
#include "svc_motor.h"
#include "svc_rs485.h"

#include "usart.h"      /* huart2 / hdma_usart2_rx */

/* CubeMX 把 DMA 句柄定义在 usart.c，也只在 stm32f4xx_it.c 里做了 extern，
   并没有生成头文件声明。这里自己声明一份 —— C 语言里重复的 extern 声明是合法的，
   将来 CubeMX 如果补上声明也不会冲突。 */
extern DMA_HandleTypeDef hdma_usart2_rx;

/* ---------------------------------------------------------------------------
 * Y42 上电自检：周期读取两台电机的位置，打印到调试串口（USART1）。
 *
 * 只做读取，不会让电机动作，用来验证整条 485 链路：
 *   方向脚切换 -> DMA+IDLE 接收 -> 帧组装 -> 应答解析 -> 地址过滤
 *
 * 功能验证完成后把 APP_Y42_POLL_TEST 改成 0 即可关闭。
 * ------------------------------------------------------------------------ */
#define APP_Y42_POLL_TEST        0
/* 采样周期调小，抓手动转轴时的连续变化 */
#define APP_Y42_POLL_PERIOD_MS   100U
/* 数值没变化时不再刷屏，但每隔这么久打一条心跳证明链路还活着 */
#define APP_Y42_HEARTBEAT_MS     5000U
#define APP_Y42_ADDR_FIRST       1U
/* 当前只接了 ID=1 这一台做测试。接上第二台之后把这里改回 2U。 */
#define APP_Y42_ADDR_LAST        1U

#if (APP_Y42_POLL_TEST != 0)
static void App_Y42PollTask(void *argument);

static const osThreadAttr_t s_y42_poll_attr = {
  .name       = "Y42Poll",
  .stack_size = 1024U,
  .priority   = (osPriority_t)osPriorityBelowNormal,
};

static void App_Y42PollTask(void *argument)
{
  /* 只记"上一次打印过的值"：变化时才输出，避免 100ms 一次刷满串口。
     addr 直接当下标用（1..APP_Y42_ADDR_LAST）。 */
  int32_t  last_pos[APP_Y42_ADDR_LAST + 1U];
  bool     seen[APP_Y42_ADDR_LAST + 1U];
  uint8_t  addr;
  uint32_t last_beat;

  (void)argument;

  for (addr = 0U; addr <= APP_Y42_ADDR_LAST; addr++)
  {
    last_pos[addr] = 0;
    seen[addr]     = false;
  }

  last_beat = HAL_GetTick();

  for (;;)
  {
    for (addr = APP_Y42_ADDR_FIRST; addr <= APP_Y42_ADDR_LAST; addr++)
    {
      int32_t pulses = 0;

      if (!SvcMotor_Y42GetPosPulses(addr, &pulses))
      {
        continue;
      }

      /* 数值变了才打印 —— 手动转轴时每一跳都能看到 */
      if ((!seen[addr]) || (pulses != last_pos[addr]))
      {
        seen[addr]     = true;
        last_pos[addr] = pulses;

        /* 角度用"百分之一度"的整数打印，避开 MicroLib 的 %f */
        log_printf("Y42", "addr=%u pos=%ld (%ld centi-deg)",
                   (unsigned)addr, (long)pulses,
                   (long)(Y42_PositionToDegrees(
                              pulses, SVC_MOTOR_Y42_FIRMWARE) * 100.0f));
      }
    }

    /* 兜底心跳：长时间没变化也要能看出链路还活着 */
    if ((HAL_GetTick() - last_beat) >= APP_Y42_HEARTBEAT_MS)
    {
      last_beat = HAL_GetTick();

      for (addr = APP_Y42_ADDR_FIRST; addr <= APP_Y42_ADDR_LAST; addr++)
      {
        if (seen[addr])
        {
          log_printf("Y42", "addr=%u heartbeat pos=%ld",
                     (unsigned)addr, (long)last_pos[addr]);
        }
      }
    }

    osDelay(APP_Y42_POLL_PERIOD_MS);
  }
}
#endif

/* ---------------------------------------------------------------------------
 * CAN 服务开关
 *
 * 当前阶段只调试 Y42（485 通道），DM3519 还没接上，所以先把 CAN 关掉：
 *   - 少一条初始化路径，排查问题时干扰更少
 *   - 日志里能明确看出"现在只有 485 在工作"
 *
 * 接上 DM3519 之后，把这里改成 1 即可恢复，不需要改任何其它代码。
 *
 * 注意：即便保持为 1，只要没有命令触发发送，CAN 也只是安静地待着，
 *       不会影响 485 —— 关掉纯粹是为了让调试阶段更干净。
 * ------------------------------------------------------------------------ */
#define APP_CAN_ENABLE           1

/* Exported functions --------------------------------------------------------*/
void App_Init(void)
{
  /* 顺序是有依赖的，不能随意调整：
     1) 业务层先就绪，因为 CAN 服务的接收回调要指向它；
     2) CAN 服务随后启动，此时设备状态缓存已经可用；
     3) TCP 服务最后启动，保证上位机连上来时整条链路已经打通。
     反过来做的后果是：上位机在初始化完成前发指令，会打到空指针上。 */

  if (!SvcMotor_Init())
  {
    log_printf("APP", "motor service init failed");
    Error_Handler();
  }

#if (APP_CAN_ENABLE != 0)
  /* CAN 初始化失败不当作致命错误：它挂掉不该影响 485 和以太网 */
  if (!SvcCan_Init(SvcMotor_OnCanFrame))
  {
    log_printf("APP", "can service init failed");
  }
#else
  log_printf("APP", "CAN disabled (APP_CAN_ENABLE=0), Y42/RS485 only");
#endif

  /* RS485 通道（Y42 步进电机）。
     注意：485 初始化失败不当作致命错误 —— 它挂掉不影响 CAN 和以太网，
     所以只告警继续跑，避免一个外设的问题把整块板子拖死。 */
  {
    UART_Config_t rs485_cfg;

    rs485_cfg.hal        = &huart2;        /* PD5 = TX, PD6 = RX */
    rs485_cfg.hdma_rx    = &hdma_usart2_rx;
    rs485_cfg.rx_queue   = NULL;           /* 由服务内部创建 */
    rs485_cfg.de_port    = GPIOD;          /* 485_EN 接在 PD4 */
    rs485_cfg.de_pin     = GPIO_PIN_4;
    rs485_cfg.de_tx_high = true;           /* 高电平 = 发送 */

    if (!SvcRs485_Init(&rs485_cfg))
    {
      log_printf("APP", "rs485 service init FAILED - Y42 不可用");
    }

#if (APP_Y42_POLL_TEST != 0)
    /* 只有 485 起来了才创建轮询任务，否则只是每秒刷一堆超时 */
    if (SvcRs485_IsReady())
    {
      if (osThreadNew(App_Y42PollTask, NULL, &s_y42_poll_attr) == NULL)
      {
        log_printf("APP", "y42 poll task create failed");
      }
    }
#endif
  }

  /* 调试串口命令行：放在最后启动，此时电机业务已经可用 */
  /* 底盘运动学先用默认配置初始化：转向基准角目前是占位值，
     实车标定出真实的直行角度后，用 Chassis_Init() 覆盖。 */
  Chassis_Init(NULL);

  if (!AppCmd_Init())
  {
    log_printf("APP", "command console init failed");
  }

  app_eth_init();

  log_printf("APP", "application ready");
}
