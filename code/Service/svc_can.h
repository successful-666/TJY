/**
  ******************************************************************************
  * @file    svc_can.h
  * @brief   CAN 服务：接收分发、异步发送、bus-off 自愈
  ******************************************************************************
  * @note    本层不解析任何设备协议。收到的帧通过注册的回调交给上层，
  *          对 CAN 一无所知的上层模块也能复用本服务。
  ******************************************************************************
  */

#ifndef __SVC_CAN_H__
#define __SVC_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "bsp_can.h"

/* Exported constants --------------------------------------------------------*/
#define SVC_CAN_RX_QUEUE_LEN        32U   /* 中断 -> 接收任务 */
#define SVC_CAN_TX_QUEUE_LEN        32U   /* 上层 -> 发送任务 */

#define SVC_CAN_TX_RETRY_MAX        20U   /* 邮箱忙时最多重试次数 */
#define SVC_CAN_TX_RETRY_DELAY_MS   1U    /* 每次重试前的等待 */

#define SVC_CAN_RX_WAIT_MS          100U  /* 接收任务的等待超时，用于顺带做巡检 */
#define SVC_CAN_STATS_LOG_MS        10000U

/* 调试阶段：每 10 秒无条件打印一次收发统计，方便确认 CAN 是否真的在收包。
   功能验证完成后改成 0，只在出现丢包/错误时才打印。 */
#define SVC_CAN_STATS_ALWAYS_LOG    1

/* Exported types ------------------------------------------------------------*/
/** 收到一帧 CAN 数据时的回调，运行在 CAN 接收任务上下文 */
typedef void (*SvcCanRxHandler_t)(const CAN_Frame_t *frame);

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  初始化 CAN 服务：建队列、启动 BSP、创建收发任务
  * @param  handler 接收回调，可为 NULL（表示暂不处理）
  * @retval true 成功
  */
bool SvcCan_Init(SvcCanRxHandler_t handler);

/**
  * @brief  异步发送一帧（入队即返回，不阻塞调用者）
  * @retval false 表示队列已满，调用方应丢弃或稍后重试
  */
bool SvcCan_Send(const CAN_Frame_t *frame);

/** @brief 获取底层实例，用于读取统计信息 */
const CAN_Inst_t *SvcCan_GetInst(void);

#ifdef __cplusplus
}
#endif

#endif /* __SVC_CAN_H__ */
