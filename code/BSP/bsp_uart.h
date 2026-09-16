/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   UART 硬件封装层：DMA+IDLE 接收、半双工方向控制发送
  ******************************************************************************
  * @note    本层只负责"把字节搬进搬出"，不解析任何协议。
  *          485 收发器的收发切换也在这里做，上层不需要关心方向引脚。
  ******************************************************************************
  */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* Exported constants --------------------------------------------------------*/
#define BSP_UART_MAX_INST       2U
#define BSP_UART_RX_SIZE        256U    /* 单帧最大长度，Modbus RTU 上限即 256 */
#define BSP_UART_TX_TIMEOUT     100U    /* 发送超时（ms） */

/* Exported types ------------------------------------------------------------*/
/** 一次 DMA 接收到的数据块，语义由上层协议决定 */
typedef struct
{
  uint16_t len;
  uint8_t  data[BSP_UART_RX_SIZE];
} UART_Frame_t;

/** 初始化参数 */
typedef struct
{
  UART_HandleTypeDef *hal;
  DMA_HandleTypeDef  *hdma_rx;
  osMessageQueueId_t  rx_queue;     /* 元素类型为 UART_Frame_t */
  GPIO_TypeDef       *de_port;      /* 485 方向脚；NULL 表示全双工，不做切换 */
  uint16_t            de_pin;
  bool                de_tx_high;   /* 发送时方向脚的电平（本板为高） */
} UART_Config_t;

/** 单个 UART 实例的句柄与运行统计 */
typedef struct
{
  UART_HandleTypeDef *hal;
  DMA_HandleTypeDef  *hdma_rx;
  osMessageQueueId_t  rx_queue;

  GPIO_TypeDef       *de_port;
  uint16_t            de_pin;
  bool                de_tx_high;

  uint8_t             dma_rx_buffer[BSP_UART_RX_SIZE];
  UART_Frame_t        isr_frame;      /* 中断里的中转缓冲，避免占用中断栈 */

  volatile uint32_t   rx_frame_count;
  volatile uint32_t   rx_drop_count;
  volatile uint32_t   tx_count;
  volatile uint32_t   tx_error_count;
  volatile uint32_t   rx_error_count;
  volatile uint32_t   rx_restart_error_count;
  volatile bool       rx_running;
  volatile bool       initialized;
} UART_Inst_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  初始化并启动 DMA+IDLE 接收
  * @note   调用前，CubeMX 里必须已经为对应串口配置好 RX 的 DMA，
  *         并且使能该串口的全局中断（IDLE 检测依赖它）
  */
bool BSP_UART_Init(UART_Inst_t *inst, const UART_Config_t *config);

/**
  * @brief  发送一段数据，自动处理 485 方向切换
  * @note   **阻塞函数**，会等到最后一个字节完全移出（TC）才返回。
  *         必须只在 485 服务任务里调用，不要放在中断或高优先级任务里。
  */
bool BSP_UART_Send(UART_Inst_t *inst, const uint8_t *data, uint16_t len);

/** @brief 重启接收（用于错误后自愈） */
bool BSP_UART_RestartRx(UART_Inst_t *inst);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H__ */
