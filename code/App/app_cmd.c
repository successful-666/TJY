/**
  ******************************************************************************
  * @file    app_cmd.c
  * @brief   调试串口命令行实现
  ******************************************************************************
  * @note    接收走 USART1 的 RXNE 中断；中断里只把字节塞进行缓冲，
  *          解析和执行都在任务里做 —— 485 收发是阻塞事务，绝不能进中断。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_cmd.h"

#include <string.h>

#include "cmsis_os.h"
#include "chassis.h"
#include "log.h"
#include "svc_motor.h"
#include "usart.h"

/* Private define ------------------------------------------------------------*/
#define CMD_LINE_MAX        64U
#define CMD_TASK_STACK      1536U
#define CMD_POLL_MS         50U

#define CMD_DEFAULT_VEL_RPM 300U
#define CMD_DEFAULT_ACC     10U

/* 静默超时：字节停这么久就把已收到的当成一整行。
   这样即使串口助手没勾"发送新行"，命令照样能执行。 */
#define CMD_IDLE_FLUSH_MS   200U

/* Private variables ---------------------------------------------------------*/
static char             s_rx_line[CMD_LINE_MAX];   /* 中断里逐字节填充 */
static volatile uint8_t s_rx_len = 0U;
static char             s_pending[CMD_LINE_MAX];   /* 凑齐的一行，交给任务 */
static volatile bool    s_line_ready = false;
static osThreadId_t     s_cmd_task = NULL;
static uint8_t          s_rx_byte = 0U;            /* HAL 单字节接收缓冲 */
static volatile uint32_t s_rx_bytes = 0U;          /* 累计收到的字节数（诊断用） */

/* 诊断用：把最近的原始字节存下来，由任务以十六进制和文本两种形式打出来。
   收到的内容如果不是预期的 ASCII 命令，看这个就能立刻知道是什么。 */
#define CMD_DBG_MAX        32U
static volatile uint8_t s_dbg[CMD_DBG_MAX];
static volatile uint8_t s_dbg_len = 0U;
static volatile uint32_t s_last_byte_tick = 0U;    /* 最后一个字节到达的时刻 */

static const osThreadAttr_t s_cmd_task_attr = {
  .name       = "Cmd",
  .stack_size = CMD_TASK_STACK,
  .priority   = (osPriority_t)osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
static void    AppCmd_Task(void *argument);
static void    AppCmd_Execute(const char *line);
static uint8_t AppCmd_ParseInt(const char *s, int32_t *out);
static const char *AppCmd_SkipSpace(const char *s);
static void    AppCmd_PrintHelp(void);
static void    AppCmd_Report(const char *what, int32_t addr, bool ok);
static void    AppCmd_FeedByte(uint8_t ch);
static void    AppCmd_StartRx(void);

/* Private functions ---------------------------------------------------------*/
static const char *AppCmd_SkipSpace(const char *s)
{
  while ((*s == ' ') || (*s == '\t')) { s++; }
  return s;
}

/** 解析十进制整数（支持正负号），返回消耗的字符数，0 表示解析失败 */
static uint8_t AppCmd_ParseInt(const char *s, int32_t *out)
{
  uint8_t n = 0U;
  int32_t v = 0;
  bool    neg = false;

  while ((*s == ' ') || (*s == '\t')) { s++; n++; }

  if (*s == '-')      { neg = true;  s++; n++; }
  else if (*s == '+') {              s++; n++; }

  if ((*s < '0') || (*s > '9'))
  {
    return 0U;
  }

  while ((*s >= '0') && (*s <= '9'))
  {
    v = (v * 10) + (int32_t)(*s - '0');
    s++;
    n++;
  }

  *out = neg ? -v : v;
  return n;
}

/** 解析十进制浮点数（支持正负号和小数点），返回消耗的字符数 */
static uint8_t AppCmd_ParseFloat(const char *s, float *out)
{
  uint8_t n = 0U;
  int32_t ip = 0;
  float   frac = 0.0f;
  float   scale = 0.1f;
  bool    neg = false;

  while ((*s == ' ') || (*s == '\t')) { s++; n++; }

  if (*s == '-')      { neg = true;  s++; n++; }
  else if (*s == '+') {              s++; n++; }

  if ((*s < '0') || (*s > '9'))
  {
    return 0U;
  }

  while ((*s >= '0') && (*s <= '9'))
  {
    ip = (ip * 10) + (int32_t)(*s - '0');
    s++;
    n++;
  }

  if (*s == '.')
  {
    s++;
    n++;

    while ((*s >= '0') && (*s <= '9'))
    {
      frac += (float)(*s - '0') * scale;
      scale *= 0.1f;
      s++;
      n++;
    }
  }

  *out = neg ? (-(float)ip - frac) : ((float)ip + frac);
  return n;
}

static void AppCmd_PrintHelp(void)
{
  log_printf("CMD", "-------- Y42 命令 --------");
  log_printf("CMD", "r <addr>           读实时位置");
  log_printf("CMD", "e <addr>           使能");
  log_printf("CMD", "d <addr>           失能");
  log_printf("CMD", "s <addr>           立即停止");
  log_printf("CMD", "z <addr>           当前位置清零");
  log_printf("CMD", "v <addr> <rpm>     速度模式，正负表示方向");
  log_printf("CMD", "p <addr> <pulses>  位置模式，相对当前位置");
  log_printf("CMD", "a <addr> <pulses>  位置模式，绝对位置");
  log_printf("CMD", "q <addr> <deg>     相对当前位置走一个角度");
  log_printf("CMD", "t <offset_deg>     转向偏移，0=回正 负=左 正=右");
  log_printf("CMD", "-- 以下大写为 CAN/DM3519 --");
  log_printf("CMD", "E <id>             使能");
  log_printf("CMD", "D <id>             失能");
  log_printf("CMD", "S <id>             速度归零");
  log_printf("CMD", "V <id> <rpm>       速度模式，正负表示方向");
  log_printf("CMD", "R <id>             显示状态");
  log_printf("CMD", "?                  显示本帮助");
  log_printf("CMD", "示例: e 1 / v 1 300 / p 1 3200 / r 1");
}

static void AppCmd_Report(const char *what, int32_t addr, bool ok)
{
  log_printf("CMD", "Y42[%ld] %s %s", (long)addr, what, ok ? "已下发" : "失败");
}

/** 把一个字节喂进行缓冲，遇到换行就整行交给任务 */
static void AppCmd_FeedByte(uint8_t ch)
{
  s_rx_bytes++;
  s_last_byte_tick = HAL_GetTick();

  if (s_dbg_len < CMD_DBG_MAX)
  {
    s_dbg[s_dbg_len] = ch;
    s_dbg_len++;
  }

  /* \r 和 \n 都当作行结束符：不同串口助手的换行设置不一样
     （\r\n / \n / \r 三种都有），只认 \n 的话用 \r 的助手会完全没反应。 */
  if ((ch == '\n') || (ch == '\r'))
  {
    if ((!s_line_ready) && (s_rx_len > 0U))
    {
      memcpy(s_pending, s_rx_line, s_rx_len);
      s_pending[s_rx_len] = '\0';
      s_line_ready = true;
      s_rx_len = 0U;
    }
  }
  else
  {
    if (s_rx_len < (CMD_LINE_MAX - 1U))
    {
      s_rx_line[s_rx_len] = (char)ch;
      s_rx_len++;
    }
  }
}

static void AppCmd_StartRx(void)
{
  /* 每次收到一个字节后都要重新挂一次，HAL 的单字节接收不是一次性的 */
  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

/**
  * @brief  解析并执行一行命令
  * @note   语法统一：单字母命令 + 电机地址 + 可选参数
  */
static void AppCmd_Execute(const char *line)
{
  const char *p = AppCmd_SkipSpace(line);
  char        cmd;
  int32_t     addr = 0;
  int32_t     arg  = 0;
  float       farg = 0.0f;
  uint8_t     n;

  if (*p == '\0')
  {
    return;
  }
  if (*p == '?')
  {
    AppCmd_PrintHelp();
    return;
  }

  cmd = *p;
  p++;

  /* 除帮助外，所有命令都紧跟一个电机地址 */
  n = AppCmd_ParseInt(p, &addr);
  if (n == 0U)
  {
    log_printf("CMD", "缺少电机地址，输入 ? 看帮助");
    return;
  }
  p = AppCmd_SkipSpace(p + n);

  switch (cmd)
  {
  case 'r':
  {
    int32_t pulses = 0;

    if (SvcMotor_Y42GetPosPulses((uint8_t)addr, &pulses))
    {
      /* 实测发现这个"实时位置"可能是累计多圈值，所以两种都显示：
         累计角度看得出电机总共转了多少，单圈角度才是机械臂的实际朝向。 */
      float total_deg = Y42_PositionToDegrees(pulses, SVC_MOTOR_Y42_FIRMWARE);
      float shaft_deg = total_deg;

      while (shaft_deg >= 360.0f) { shaft_deg -= 360.0f; }
      while (shaft_deg < 0.0f)    { shaft_deg += 360.0f; }

      log_printf("CMD", "Y42[%ld] 累计 %.1f 度 | 单圈 %.2f 度 | 原始 %ld",
                 (long)addr, (double)total_deg, (double)shaft_deg,
                 (long)pulses);
    }
    else
    {
      log_printf("CMD", "Y42[%ld] 读取失败", (long)addr);
    }
    return;
  }

  case 'e':
    AppCmd_Report("使能", addr, SvcMotor_Y42Enable((uint8_t)addr, true));
    return;

  case 'd':
    AppCmd_Report("失能", addr, SvcMotor_Y42Enable((uint8_t)addr, false));
    return;

  case 's':
    AppCmd_Report("停止", addr, SvcMotor_Y42Stop((uint8_t)addr));
    return;

  case 'z':
    AppCmd_Report("位置清零", addr, SvcMotor_Y42ResetCurPos((uint8_t)addr));
    return;

  case 'v':
  case 'p':
  case 'a':
  case 'q':
  case 't':
  case 'V':
    /* 这些还需要第二个参数；角度允许小数，所以统一按浮点解析 */
    n = AppCmd_ParseFloat(p, &farg);
    if (n == 0U)
    {
      log_printf("CMD", "缺少参数，输入 ? 看帮助");
      return;
    }
    break;

  /* ---- 以下大写字母 = CAN / DM3519 ---- */
  case 'E':
    AppCmd_Report("DM 使能", addr, SvcMotor_DmEnable((uint8_t)addr, true));
    return;

  case 'D':
    AppCmd_Report("DM 失能", addr, SvcMotor_DmEnable((uint8_t)addr, false));
    return;

  case 'S':
    AppCmd_Report("DM 速度归零", addr,
                  SvcMotor_DmSetSpeedRpm((uint8_t)addr, 0));
    return;

  case 'R':
  {
    const DM3519_t *dm = SvcMotor_GetDm((uint8_t)addr);

    if (dm == NULL)
    {
      log_printf("CMD", "DM 的 ID 只能是 1~%u",
                 (unsigned)SVC_MOTOR_DM_COUNT);
      return;
    }

    /* 注意：DM3519 只有在收到命令后才会回帧，
       所以"离线"通常意味着还没给它发过任何命令。 */
    log_printf("CMD", "DM%ld %s err=%u 位置=%.3f rad 速度=%.1f rpm",
               (long)addr, dm->online ? "在线" : "离线",
               (unsigned)dm->error_code,
               (double)dm->position_rad, (double)dm->velocity_rpm);
    log_printf("CMD", "     力矩=%.2f Nm MOS=%u 转子=%u 反馈帧数=%u",
               (double)dm->torque_nm,
               (unsigned)dm->mos_temperature,
               (unsigned)dm->rotor_temperature,
               (unsigned)dm->rx_count);
    return;
  }

  default:
    log_printf("CMD", "未知命令 '%c'，输入 ? 看帮助", cmd);
    return;
  }

  /* ---- 带参数的命令 ---- */
  if (cmd == 'q')
  {
    AppCmd_Report("相对走位(度)", addr,
                  SvcMotor_Y42MoveDegrees((uint8_t)addr, farg,
                                          CMD_DEFAULT_VEL_RPM,
                                          CMD_DEFAULT_ACC));
    return;
  }

  if (cmd == 't')
  {
    /* 转向：用底盘配置里的基准角和镜像系数算出两台转向电机的目标角 */
    Chassis_Output_t        out;
    const Chassis_Config_t *cfg = Chassis_GetConfig();
    uint8_t                 i;

    if (cfg == NULL)
    {
      log_printf("CMD", "底盘未初始化");
      return;
    }
    if (!Chassis_Mix(0, 0, (int16_t)farg, &out))
    {
      log_printf("CMD", "转向解算失败");
      return;
    }

    for (i = 0U; i < CHASSIS_STEER_COUNT; i++)
    {
      log_printf("CMD", "转向%u -> addr=%u 目标 %.3f 度",
                 (unsigned)(i + 1U), (unsigned)cfg->steer_y42_addr[i],
                 (double)out.steer_angle_deg[i]);

      (void)SvcMotor_Y42SetPosDegrees(cfg->steer_y42_addr[i],
                                      (float)out.steer_angle_deg[i],
                                      cfg->steer_vel_rpm,
                                      cfg->steer_acc,
                                      Y42_MOVE_ABSOLUTE);
    }
    return;
  }

  arg = (int32_t)farg;

  if (cmd == 'V')
  {
    /* DM3519 速度模式：协议里速度是 float32 rad/s，量程 ±30 rad/s。
       换算成转速约 ±286 rpm，超出部分限幅，避免下发无效值。 */
    int32_t rpm = arg;
    const int32_t rpm_max = 286;

    if (rpm > rpm_max)       { rpm = rpm_max; }
    else if (rpm < -rpm_max) { rpm = -rpm_max; }

    AppCmd_Report("DM 速度设定", addr,
                  SvcMotor_DmSetSpeedRpm((uint8_t)addr, (int16_t)rpm));
    return;
  }

  if (cmd == 'v')
  {
    /* 速度模式：正负号决定方向，幅值限幅到协议上限 */
    uint8_t dir = (arg < 0) ? Y42_DIR_CCW : Y42_DIR_CW;
    int32_t vel = (arg < 0) ? -arg : arg;

    if (vel > (int32_t)Y42_VEL_MAX_RPM)
    {
      vel = (int32_t)Y42_VEL_MAX_RPM;
      log_printf("CMD", "速度超上限，已限幅到 %ld rpm", (long)vel);
    }

    AppCmd_Report("速度设定", addr,
                  SvcMotor_Y42SetSpeedRpm((uint8_t)addr, dir,
                                          (uint16_t)vel, CMD_DEFAULT_ACC));
  }
  else if (cmd == 'p')
  {
    AppCmd_Report("相对走位", addr,
                  SvcMotor_Y42SetPosPulses((uint8_t)addr, arg,
                                           CMD_DEFAULT_VEL_RPM,
                                           CMD_DEFAULT_ACC,
                                           Y42_MOVE_REL_NOW));
  }
  else /* 'a' */
  {
    AppCmd_Report("绝对走位", addr,
                  SvcMotor_Y42SetPosPulses((uint8_t)addr, arg,
                                           CMD_DEFAULT_VEL_RPM,
                                           CMD_DEFAULT_ACC,
                                           Y42_MOVE_ABSOLUTE));
  }
}

static void AppCmd_Task(void *argument)
{
  char line[CMD_LINE_MAX];

  (void)argument;

  log_printf("CMD", "命令行就绪，输入 ? 查看命令");

  for (;;)
  {
    /* 静默超时补一个"虚拟换行"：终端不发 \r\n 时也能执行命令 */
    if ((!s_line_ready) && (s_rx_len > 0U) &&
        ((HAL_GetTick() - s_last_byte_tick) > CMD_IDLE_FLUSH_MS))
    {
      memcpy(s_pending, s_rx_line, s_rx_len);
      s_pending[s_rx_len] = '\0';
      s_line_ready = true;
      s_rx_len = 0U;
    }

    /* 防御性重挂：如果因为溢出等错误导致 HAL 接收停了，
       这里把错误标志清掉并重新挂上，避免命令行彻底失聪。 */
    if (huart1.RxState == HAL_UART_STATE_READY)
    {
      __HAL_UART_CLEAR_OREFLAG(&huart1);
      AppCmd_StartRx();
    }

    if (s_line_ready)
    {
      memcpy(line, s_pending, sizeof(line));
      line[CMD_LINE_MAX - 1U] = '\0';
      s_line_ready = false;

      /* 先把收到的原样回显一遍：这样"收没收到"一眼就能看出来，
         不用去猜是命令写错了还是根本没收到字节。 */
      log_printf("CMD", "> %s", line);

      AppCmd_Execute(line);
    }

    /* 诊断：只要收到过字节就报告一次累计值。
       打字时这个数字不涨 = 板子根本没收到，问题在接线或串口设置；
       数字在涨但命令不执行 = 收到的是别的内容（换行符/波特率/数据位）。 */
    {
      static uint32_t last_reported = 0U;

      if (s_rx_bytes != last_reported)
      {
        static const char hexdig[] = "0123456789ABCDEF";
        char     hex[(3U * CMD_DBG_MAX) + 1U];
        char     txt[CMD_DBG_MAX + 1U];
        uint8_t  n = s_dbg_len;
        uint8_t  i;
        uint16_t k = 0U;

        last_reported = s_rx_bytes;

        for (i = 0U; i < n; i++)
        {
          uint8_t b = s_dbg[i];

          hex[k] = hexdig[(b >> 4) & 0x0FU]; k++;
          hex[k] = hexdig[b & 0x0FU];        k++;
          hex[k] = ' ';                      k++;

          /* 不可打印字符用 '.' 代替，避免把终端搞乱 */
          txt[i] = ((b >= 0x20U) && (b < 0x7FU)) ? (char)b : '.';
        }
        hex[k] = '\0';
        txt[n] = '\0';

        log_printf("CMD", "RX 累计 %u 字节", (unsigned)last_reported);
        if (n > 0U)
        {
          log_printf("CMD", "  HEX: %s", hex);
          log_printf("CMD", "  TXT: %s", txt);
        }
        s_dbg_len = 0U;
      }
    }

    osDelay(CMD_POLL_MS);
  }
}

/* Exported functions --------------------------------------------------------*/
bool AppCmd_Init(void)
{
  /* USART1 的中断和 NVIC 已经由 CubeMX 配好（优先级 5），
     这里只需要用 HAL 的标准接口挂上单字节接收。 */
  AppCmd_StartRx();

  s_cmd_task = osThreadNew(AppCmd_Task, NULL, &s_cmd_task_attr);

  return (s_cmd_task != NULL);
}

/**
  * @brief HAL 单字节接收完成回调（中断上下文）
  * @note  只做"存字节 + 重新挂接收"，绝不在这里执行命令。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    AppCmd_FeedByte(s_rx_byte);
    AppCmd_StartRx();
  }
}
