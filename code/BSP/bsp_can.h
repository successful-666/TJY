/**
  ******************************************************************************
  * @file    bsp_can.h
  * @brief   CAN 硬件封装层：滤波器、中断接收、非阻塞发送、bus-off 恢复
  ******************************************************************************
  * @note    本层是整个工程中唯一直接操作 CAN 外设的地方。
  *          上层（Service / Device）只使用 CAN_Frame_t，不接触 HAL。
  ******************************************************************************
  */

#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* Exported constants --------------------------------------------------------*/
#define BSP_CAN_MAX_INST          2U

/* Exported types ------------------------------------------------------------*/
/** 框架内部统一使用的 CAN 帧，同时兼容标准帧和扩展帧 */
typedef struct
{
  uint32_t id;          /* 标准帧 11 位 / 扩展帧 29 位 */
  uint8_t  ide;         /* CAN_ID_STD / CAN_ID_EXT */
  uint8_t  rtr;         /* CAN_RTR_DATA / CAN_RTR_REMOTE */
  uint8_t  dlc;         /* 0 ~ 8 */
  uint8_t  data[8];
} CAN_Frame_t;

/** 发送结果。BUSY 表示三个邮箱都占着，调用方应稍后重试 */
typedef enum
{
  BSP_CAN_OK = 0,
  BSP_CAN_BUSY,
  BSP_CAN_INVALID,
  BSP_CAN_ERROR
} BSP_CAN_Status_t;

/** 单个 CAN 实例的句柄与运行统计 */
typedef struct
{
  CAN_HandleTypeDef   *hal;
  osMessageQueueId_t   rx_queue;        /* 中断 -> 任务 的接收队列 */
  volatile uint32_t    rx_count;
  volatile uint32_t    rx_drop_count;   /* 队列满导致丢弃的帧数 */
  volatile uint32_t    tx_count;
  volatile uint32_t    tx_busy_count;
  volatile uint32_t    error_count;
  volatile uint32_t    busoff_count;
  volatile bool        initialized;
} CAN_Inst_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  配置全通滤波器、启动 CAN、开启 FIFO0 接收通知
  * @param  inst    实例句柄
  * @param  hal     CubeMX 生成的 CAN 句柄
  * @param  rx_queue 中断投递用的消息队列，元素类型为 CAN_Frame_t
  * @retval true 全部成功；任何一步失败都会回滚并返回 false
  */
bool BSP_CAN_Init(CAN_Inst_t *inst,
                  CAN_HandleTypeDef *hal,
                  osMessageQueueId_t rx_queue);

/**
  * @brief  非阻塞发送一帧
  * @retval BSP_CAN_OK      已写入邮箱
  *         BSP_CAN_BUSY    邮箱满，可重试
  *         BSP_CAN_INVALID 参数非法
  *         BSP_CAN_ERROR   外设报错
  */
BSP_CAN_Status_t BSP_CAN_Send(CAN_Inst_t *inst, const CAN_Frame_t *frame);

/**
  * @brief  检测并恢复 bus-off
  * @note   AutoBusOff 关闭时 bxCAN 会停在 bus-off 状态，必须由软件
  *         主动进入再退出初始化模式才能恢复。该函数应在任务上下文周期调用。
  * @retval true 本次执行了恢复动作
  */
bool BSP_CAN_RecoverBusOff(CAN_Inst_t *inst);

/**
  * @brief  重启接收（用于接收被意外关闭或出错后的自愈）
  */
bool BSP_CAN_RestartRx(CAN_Inst_t *inst);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_H__ */
