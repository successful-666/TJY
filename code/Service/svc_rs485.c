/**
  ******************************************************************************
  * @file    svc_rs485.c
  * @brief   RS485 传输服务实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "svc_rs485.h"
#include "log.h"

#include <string.h>

/* Private variables ---------------------------------------------------------*/
static UART_Inst_t         s_uart;
static osMessageQueueId_t  s_rx_queue = NULL;
static osMutexId_t         s_bus_lock = NULL;
static bool                s_ready    = false;
static uint32_t            s_last_timeout_log = 0U;

/* 查询超时的日志节流间隔：电机没接时每秒会有多条超时，
   不限流会把调试串口刷爆，反而看不到别的信息。 */
#define SVC_RS485_TIMEOUT_LOG_MS   5000U

/* Private functions ---------------------------------------------------------*/
/** 丢掉队列里滞留的旧数据，避免上一次的残留被当成本次的应答 */
static void DrainRxQueue(void)
{
  UART_Frame_t stale;

  while (osMessageQueueGet(s_rx_queue, &stale, NULL, 0U) == osOK)
  {
    /* 只是丢弃 */
  }
}

/* Exported functions --------------------------------------------------------*/
bool SvcRs485_Init(const UART_Config_t *config)
{
  UART_Config_t cfg;

  if (config == NULL)
  {
    return false;
  }

  /* 先建队列和锁，再启动 DMA 接收：
     否则 DMA 一旦启动，首帧到达时无处投递，会直接被丢掉 */
  s_rx_queue = osMessageQueueNew(SVC_RS485_RX_QUEUE_LEN,
                                 sizeof(UART_Frame_t), NULL);
  if (s_rx_queue == NULL)
  {
    log_printf("485", "rx queue create failed");
    return false;
  }

  s_bus_lock = osMutexNew(NULL);
  if (s_bus_lock == NULL)
  {
    log_printf("485", "bus mutex create failed");
    return false;
  }

  /* 队列是服务内部创建的，这里补进配置再交给 BSP */
  cfg         = *config;
  cfg.rx_queue = s_rx_queue;

  if (!BSP_UART_Init(&s_uart, &cfg))
  {
    log_printf("485", "BSP init failed (DMA 配好了吗?)");
    return false;
  }

  s_ready = true;
  log_printf("485", "service up (addr filter off, q=%u)",
             (unsigned)SVC_RS485_RX_QUEUE_LEN);
  return true;
}

bool SvcRs485_IsReady(void)
{
  return s_ready;
}

const UART_Inst_t *SvcRs485_GetInst(void)
{
  return &s_uart;
}

bool SvcRs485_Send(const uint8_t *frame, uint8_t len)
{
  bool ok;

  if ((!s_ready) || (frame == NULL) || (len == 0U))
  {
    return false;
  }

  /* 等总线控制权。超时而不是死等 —— 万一某个调用者卡住，
     也不能让整条 485 链路跟着一起僵住。 */
  if (osMutexAcquire(s_bus_lock, SVC_RS485_LOCK_TIMEOUT_MS) != osOK)
  {
    log_printf("485", "bus busy, send dropped");
    return false;
  }

  DrainRxQueue();
  ok = BSP_UART_Send(&s_uart, frame, len);

  (void)osMutexRelease(s_bus_lock);
  return ok;
}

bool SvcRs485_Query(const uint8_t *frame, uint8_t len, uint8_t expect_addr,
                    uint8_t *reply, uint16_t reply_cap, uint16_t *reply_len,
                    uint32_t timeout_ms)
{
  UART_Frame_t incoming;
  uint32_t     deadline;
  bool         found = false;

  if ((!s_ready) || (frame == NULL) || (len == 0U) ||
      (reply == NULL) || (reply_len == NULL) || (reply_cap == 0U))
  {
    return false;
  }

  if (osMutexAcquire(s_bus_lock, SVC_RS485_LOCK_TIMEOUT_MS) != osOK)
  {
    log_printf("485", "bus busy, query dropped");
    return false;
  }

  /* 清掉上次遗留，再发新命令 */
  DrainRxQueue();

  if (!BSP_UART_Send(&s_uart, frame, len))
  {
    (void)osMutexRelease(s_bus_lock);
    return false;
  }

  deadline = HAL_GetTick() + timeout_ms;

  while ((int32_t)(HAL_GetTick() - deadline) < 0)
  {
    if (osMessageQueueGet(s_rx_queue, &incoming, NULL, timeout_ms) != osOK)
    {
      break;                                   /* 超时，退出等待 */
    }

    /* 总线上可能混有其它设备的报文或上一轮的残留：
       只看地址字节是否匹配，不匹配就丢掉继续等。 */
    if (incoming.len == 0U)
    {
      continue;
    }
    if (incoming.data[0] != expect_addr)
    {
      continue;
    }

    *reply_len = (incoming.len < reply_cap) ? incoming.len : reply_cap;
    memcpy(reply, incoming.data, *reply_len);
    found = true;
    break;
  }

  (void)osMutexRelease(s_bus_lock);

  if (!found)
  {
    /* 限流：同一类超时最多每 5 秒打一条 */
    if ((HAL_GetTick() - s_last_timeout_log) >= SVC_RS485_TIMEOUT_LOG_MS)
    {
      s_last_timeout_log = HAL_GetTick();
      log_printf("485", "query timeout, addr=%u (check wiring/power/baud)",
                 (unsigned)expect_addr);
    }
  }

  return found;
}
