/**
  ******************************************************************************
  * @file    bsp_can.c
  * @brief   CAN 硬件封装层实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"

#include <string.h>

/* Private define ------------------------------------------------------------*/
/* 一次中断最多搬多少帧，防止在异常情况下长时间占用中断 */
#define BSP_CAN_ISR_DRAIN_MAX     16U

/* Private variables ---------------------------------------------------------*/
static CAN_Inst_t *s_instances[BSP_CAN_MAX_INST];
static uint8_t     s_instance_count = 0U;

/* Private functions ---------------------------------------------------------*/
/* HAL 回调只带句柄，用注册表反查对应的框架实例 */
static CAN_Inst_t *FindInstance(CAN_HandleTypeDef *hal)
{
  uint8_t i;

  for (i = 0U; i < s_instance_count; i++)
  {
    if (s_instances[i]->hal == hal)
    {
      return s_instances[i];
    }
  }

  return NULL;
}

/* Exported functions --------------------------------------------------------*/
bool BSP_CAN_Init(CAN_Inst_t *inst,
                  CAN_HandleTypeDef *hal,
                  osMessageQueueId_t rx_queue)
{
  CAN_FilterTypeDef filter;

  if ((inst == NULL) || (hal == NULL) || (rx_queue == NULL))
  {
    return false;
  }
  if (s_instance_count >= BSP_CAN_MAX_INST)
  {
    return false;
  }

  memset(inst, 0, sizeof(*inst));
  inst->hal      = hal;
  inst->rx_queue = rx_queue;

  /* 全通滤波器：标准帧和扩展帧都收进 FIFO0，具体分发交给任务层。
     掩码全 0 表示"不比较任何位"，即全部通过。 */
  memset(&filter, 0, sizeof(filter));
  filter.FilterBank           = 0U;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh         = 0U;
  filter.FilterIdLow          = 0U;
  filter.FilterMaskIdHigh     = 0U;
  filter.FilterMaskIdLow      = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation     = ENABLE;

  if (HAL_CAN_ConfigFilter(hal, &filter) != HAL_OK)
  {
    inst->error_count++;
    return false;
  }

  if (HAL_CAN_Start(hal) != HAL_OK)
  {
    inst->error_count++;
    return false;
  }

  if (HAL_CAN_ActivateNotification(hal, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    (void)HAL_CAN_Stop(hal);
    inst->error_count++;
    return false;
  }

  s_instances[s_instance_count] = inst;
  s_instance_count++;
  inst->initialized = true;

  return true;
}

BSP_CAN_Status_t BSP_CAN_Send(CAN_Inst_t *inst, const CAN_Frame_t *frame)
{
  CAN_TxHeaderTypeDef header;
  uint32_t mailbox;

  if ((inst == NULL) || (frame == NULL) || (!inst->initialized))
  {
    return BSP_CAN_INVALID;
  }
  if (frame->dlc > 8U)
  {
    return BSP_CAN_INVALID;
  }

  /* 邮箱满时立即返回，绝不在这里等待 —— 等待会阻塞整个发送任务 */
  if (HAL_CAN_GetTxMailboxesFreeLevel(inst->hal) == 0U)
  {
    inst->tx_busy_count++;
    return BSP_CAN_BUSY;
  }

  memset(&header, 0, sizeof(header));
  header.IDE               = frame->ide;
  header.RTR               = frame->rtr;
  header.DLC               = frame->dlc;
  header.TransmitGlobalTime = DISABLE;

  if (frame->ide == CAN_ID_STD)
  {
    if (frame->id > 0x7FFU)
    {
      return BSP_CAN_INVALID;
    }
    header.StdId = frame->id;
  }
  else if (frame->ide == CAN_ID_EXT)
  {
    if (frame->id > 0x1FFFFFFFU)
    {
      return BSP_CAN_INVALID;
    }
    header.ExtId = frame->id;
  }
  else
  {
    return BSP_CAN_INVALID;
  }

  if (HAL_CAN_AddTxMessage(inst->hal, &header,
                           (uint8_t *)frame->data, &mailbox) != HAL_OK)
  {
    inst->error_count++;
    return BSP_CAN_ERROR;
  }

  inst->tx_count++;
  return BSP_CAN_OK;
}

bool BSP_CAN_RecoverBusOff(CAN_Inst_t *inst)
{
  if ((inst == NULL) || (!inst->initialized))
  {
    return false;
  }

  /* 直接读 ESR 的 BOFF 位，比依赖 HAL 的错误码更可靠 */
  if ((inst->hal->Instance->ESR & CAN_ESR_BOFF) == 0U)
  {
    return false;
  }

  inst->busoff_count++;

  /* bxCAN 的恢复流程：进入初始化模式（清 BOFF）-> 退出 -> 重新开中断 */
  (void)HAL_CAN_Stop(inst->hal);

  if (HAL_CAN_Start(inst->hal) != HAL_OK)
  {
    inst->error_count++;
    return false;
  }

  if (HAL_CAN_ActivateNotification(inst->hal,
                                   CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    inst->error_count++;
    return false;
  }

  return true;
}

bool BSP_CAN_RestartRx(CAN_Inst_t *inst)
{
  if ((inst == NULL) || (!inst->initialized))
  {
    return false;
  }

  if (HAL_CAN_ActivateNotification(inst->hal,
                                   CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    inst->error_count++;
    return false;
  }

  return true;
}

/* HAL callbacks -------------------------------------------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_Inst_t *inst = FindInstance(hcan);
  CAN_RxHeaderTypeDef header;
  CAN_Frame_t frame;
  uint8_t drained = 0U;

  if ((inst == NULL) || (inst->rx_queue == NULL))
  {
    return;
  }

  /* 一次中断尽量排空硬件 FIFO，但要有上限，避免异常时卡在中断里。
     中断中只做"取帧 + 入队"，绝不解析协议、绝不打印日志。 */
  while ((HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) &&
         (drained < BSP_CAN_ISR_DRAIN_MAX))
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                             &header, frame.data) != HAL_OK)
    {
      inst->error_count++;
      break;
    }

    frame.ide = (uint8_t)header.IDE;
    frame.rtr = (uint8_t)header.RTR;
    frame.dlc = (uint8_t)header.DLC;
    frame.id  = (header.IDE == CAN_ID_STD) ? header.StdId : header.ExtId;

    /* timeout = 0：队列满时立即返回失败，中断里不允许阻塞 */
    if (osMessageQueuePut(inst->rx_queue, &frame, 0U, 0U) == osOK)
    {
      inst->rx_count++;
    }
    else
    {
      inst->rx_drop_count++;
    }

    drained++;
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  CAN_Inst_t *inst = FindInstance(hcan);

  if (inst != NULL)
  {
    inst->error_count++;
  }
}
