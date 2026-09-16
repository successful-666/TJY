/**
  ******************************************************************************
  * @file    svc_can.c
  * @brief   CAN 服务实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "svc_can.h"
#include "can.h"
#include "log.h"

/* Private variables ---------------------------------------------------------*/
static CAN_Inst_t          s_can;
static osMessageQueueId_t  s_rx_queue  = NULL;
static osMessageQueueId_t  s_tx_queue  = NULL;
static osThreadId_t        s_rx_task   = NULL;
static osThreadId_t        s_tx_task   = NULL;
static SvcCanRxHandler_t   s_rx_handler = NULL;
static uint32_t            s_last_stats_tick = 0U;

/* Private function prototypes -----------------------------------------------*/
static void SvcCan_RxTask(void *argument);
static void SvcCan_TxTask(void *argument);

/* Task attributes -----------------------------------------------------------*/
static const osThreadAttr_t s_rx_task_attr = {
  .name       = "CanRx",
  .stack_size = 1024U,
  .priority   = (osPriority_t)osPriorityAboveNormal,
};

static const osThreadAttr_t s_tx_task_attr = {
  .name       = "CanTx",
  .stack_size = 1024U,
  .priority   = (osPriority_t)osPriorityNormal,
};

/* Exported functions --------------------------------------------------------*/
bool SvcCan_Init(SvcCanRxHandler_t handler)
{
  s_rx_handler = handler;

  /* 先建队列再启动 CAN：否则第一帧到来时无处投递，会直接丢掉 */
  s_rx_queue = osMessageQueueNew(SVC_CAN_RX_QUEUE_LEN,
                                 sizeof(CAN_Frame_t), NULL);
  if (s_rx_queue == NULL)
  {
    log_printf("CAN", "rx queue create failed");
    return false;
  }

  s_tx_queue = osMessageQueueNew(SVC_CAN_TX_QUEUE_LEN,
                                 sizeof(CAN_Frame_t), NULL);
  if (s_tx_queue == NULL)
  {
    log_printf("CAN", "tx queue create failed");
    return false;
  }

  if (!BSP_CAN_Init(&s_can, &hcan1, s_rx_queue))
  {
    log_printf("CAN", "BSP init failed");
    return false;
  }

  s_rx_task = osThreadNew(SvcCan_RxTask, NULL, &s_rx_task_attr);
  s_tx_task = osThreadNew(SvcCan_TxTask, NULL, &s_tx_task_attr);

  if ((s_rx_task == NULL) || (s_tx_task == NULL))
  {
    log_printf("CAN", "task create failed");
    return false;
  }

  log_printf("CAN", "service up (rx q=%u, tx q=%u)",
             (unsigned)SVC_CAN_RX_QUEUE_LEN, (unsigned)SVC_CAN_TX_QUEUE_LEN);
  return true;
}

bool SvcCan_Send(const CAN_Frame_t *frame)
{
  if ((frame == NULL) || (s_tx_queue == NULL))
  {
    return false;
  }

  return (osMessageQueuePut(s_tx_queue, frame, 0U, 0U) == osOK);
}

const CAN_Inst_t *SvcCan_GetInst(void)
{
  return &s_can;
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  接收任务：阻塞等队列，有帧就交给上层；空闲时顺带做 bus-off 巡检
  */
static void SvcCan_RxTask(void *argument)
{
  CAN_Frame_t frame;
  osStatus_t  status;

  (void)argument;

  for (;;)
  {
    /* 用带超时的等待，这样即使总线上长期无报文也能周期性地做健康检查 */
    status = osMessageQueueGet(s_rx_queue, &frame, NULL, SVC_CAN_RX_WAIT_MS);

    if (status == osOK)
    {
      if (s_rx_handler != NULL)
      {
        s_rx_handler(&frame);
      }
    }

    /* 无论有没有收到帧都检查一次：bus-off 是静默故障，不检查就会一直不通 */
    if (BSP_CAN_RecoverBusOff(&s_can))
    {
      log_printf("CAN", "bus-off recovered (count=%u)",
                 (unsigned)s_can.busoff_count);
    }

    /* 周期打印一次健康统计，便于长时间运行后回溯问题 */
    if ((HAL_GetTick() - s_last_stats_tick) >= SVC_CAN_STATS_LOG_MS)
    {
      s_last_stats_tick = HAL_GetTick();

      /* 调试期无条件打印：rx=0 说明一个包都没收到，是波特率/接线问题；
         rx 在涨说明接收链路是通的。这条日志是 CAN 是否工作最直接的证据。 */
      if ((SVC_CAN_STATS_ALWAYS_LOG != 0) ||
          (s_can.rx_drop_count != 0U) || (s_can.error_count != 0U) ||
          (s_can.tx_busy_count != 0U))
      {
        log_printf("CAN", "stat rx=%u drop=%u tx=%u busy=%u err=%u bof=%u",
                   (unsigned)s_can.rx_count,
                   (unsigned)s_can.rx_drop_count,
                   (unsigned)s_can.tx_count,
                   (unsigned)s_can.tx_busy_count,
                   (unsigned)s_can.error_count,
                   (unsigned)s_can.busoff_count);
      }
    }
  }
}

/**
  * @brief  发送任务：串行化所有发送请求，邮箱忙时有限次重试
  * @note   重试上限是必要的 —— 无上限的重试会在总线卡死时把发送任务永久挂住
  */
static void SvcCan_TxTask(void *argument)
{
  CAN_Frame_t      frame;
  BSP_CAN_Status_t result;
  uint8_t          retry;

  (void)argument;

  for (;;)
  {
    if (osMessageQueueGet(s_tx_queue, &frame, NULL, osWaitForever) != osOK)
    {
      continue;
    }

    for (retry = 0U; retry < SVC_CAN_TX_RETRY_MAX; retry++)
    {
      result = BSP_CAN_Send(&s_can, &frame);

      if (result == BSP_CAN_OK)
      {
        break;
      }
      if (result != BSP_CAN_BUSY)
      {
        /* 参数非法或外设报错，重试没有意义，直接丢弃并记账 */
        log_printf("CAN", "send failed id=0x%03X result=%u",
                   (unsigned)frame.id, (unsigned)result);
        break;
      }

      osDelay(SVC_CAN_TX_RETRY_DELAY_MS);
    }

    if (retry >= SVC_CAN_TX_RETRY_MAX)
    {
      log_printf("CAN", "tx timeout, drop id=0x%03X", (unsigned)frame.id);
    }
  }
}
