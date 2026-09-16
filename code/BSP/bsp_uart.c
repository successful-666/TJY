/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   UART 硬件封装层实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"

#include <string.h>

/* Private variables ---------------------------------------------------------*/
static UART_Inst_t *s_instances[BSP_UART_MAX_INST];
static uint8_t      s_instance_count = 0U;

/* Private functions ---------------------------------------------------------*/
static UART_Inst_t *FindInstance(UART_HandleTypeDef *hal)
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

/** 启动一次 DMA 接收。IDLE 到来或缓冲区满时触发 RxEvent 回调 */
static bool StartRx(UART_Inst_t *inst)
{
  if ((inst == NULL) || (inst->hal == NULL) || (inst->hdma_rx == NULL))
  {
    return false;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(inst->hal, inst->dma_rx_buffer,
                                   BSP_UART_RX_SIZE) != HAL_OK)
  {
    inst->rx_running = false;
    inst->rx_restart_error_count++;
    return false;
  }

  /* 半传输中断不是协议边界，没有意义还会增加中断负载，关掉它 */
  __HAL_DMA_DISABLE_IT(inst->hdma_rx, DMA_IT_HT);

  inst->rx_running = true;
  return true;
}

/* Exported functions --------------------------------------------------------*/
bool BSP_UART_Init(UART_Inst_t *inst, const UART_Config_t *config)
{
  if ((inst == NULL) || (config == NULL) ||
      (config->hal == NULL) || (config->hdma_rx == NULL) ||
      (config->rx_queue == NULL))
  {
    return false;
  }
  if (s_instance_count >= BSP_UART_MAX_INST)
  {
    return false;
  }

  memset(inst, 0, sizeof(*inst));
  inst->hal         = config->hal;
  inst->hdma_rx     = config->hdma_rx;
  inst->rx_queue    = config->rx_queue;
  inst->de_port     = config->de_port;
  inst->de_pin      = config->de_pin;
  inst->de_tx_high  = config->de_tx_high;

  /* 上电默认置为接收方向：绝不能在上电瞬间抢占总线 */
  if (inst->de_port != NULL)
  {
    HAL_GPIO_WritePin(inst->de_port, inst->de_pin,
                      inst->de_tx_high ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }

  s_instances[s_instance_count] = inst;
  s_instance_count++;
  inst->initialized = true;

  return StartRx(inst);
}

bool BSP_UART_Send(UART_Inst_t *inst, const uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status;

  if ((inst == NULL) || (data == NULL) || (len == 0U) ||
      (!inst->initialized))
  {
    return false;
  }

  /* 切到发送方向。
     本板的 RE# 和 DE 由同一个引脚驱动，切过去之后接收器也被关掉，
     所以不会收到自己发出去的回波，不需要额外的回显过滤。 */
  if (inst->de_port != NULL)
  {
    HAL_GPIO_WritePin(inst->de_port, inst->de_pin,
                      inst->de_tx_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  /* 阻塞发送。HAL_UART_Transmit 会一直等到 TC（发送完成）标志，
     保证最后一个字节完全移出总线后才切回接收 —— 提前切会截断帧尾。 */
  status = HAL_UART_Transmit(inst->hal, (uint8_t *)data, len,
                             BSP_UART_TX_TIMEOUT);

  /* 无论成败都要切回接收方向，否则总线会被一直占着 */
  if (inst->de_port != NULL)
  {
    HAL_GPIO_WritePin(inst->de_port, inst->de_pin,
                      inst->de_tx_high ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }

  if (status != HAL_OK)
  {
    inst->tx_error_count++;
    return false;
  }

  inst->tx_count++;
  return true;
}

bool BSP_UART_RestartRx(UART_Inst_t *inst)
{
  if ((inst == NULL) || (!inst->initialized))
  {
    return false;
  }

  return StartRx(inst);
}

/* HAL callbacks -------------------------------------------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  UART_Inst_t *inst = FindInstance(huart);

  if (inst == NULL)
  {
    return;
  }

  inst->rx_running = false;

  /* 中断里只做"拷贝 + 入队"，绝不解析协议、绝不打印日志。
     中转缓冲放在实例结构体里，避免在中断栈上开 256 字节的局部变量。 */
  if ((size > 0U) && (size <= BSP_UART_RX_SIZE))
  {
    inst->isr_frame.len = size;
    memcpy(inst->isr_frame.data, inst->dma_rx_buffer, size);

    if (osMessageQueuePut(inst->rx_queue, &inst->isr_frame, 0U, 0U) == osOK)
    {
      inst->rx_frame_count++;
    }
    else
    {
      inst->rx_drop_count++;
    }
  }

  (void)StartRx(inst);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  UART_Inst_t *inst = FindInstance(huart);

  if (inst == NULL)
  {
    return;
  }

  inst->rx_running = false;
  inst->rx_error_count++;

  /* 一次错误（比如噪声导致的帧错误）不能让接收永久停摆，
     清理 HAL 状态后自动重新开始接收。 */
  (void)HAL_UART_AbortReceive(huart);
  (void)StartRx(inst);
}
