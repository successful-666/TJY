/**
  ******************************************************************************
  * @file    svc_rs485.h
  * @brief   RS485 传输服务：总线互斥、请求/应答事务
  ******************************************************************************
  * @note    485 是半双工共享总线，"谁在什么时候说话"必须串行化，
  *          否则两路命令会互相踩踏。本层用一把互斥量保证任意时刻
  *          只有一个调用者在总线上收发。
  *
  *          本层不解析设备协议，只负责"把帧发出去、把回复收回来"。
  *          Y42 的协议解析在 Device/y42.c。
  ******************************************************************************
  */

#ifndef __SVC_RS485_H__
#define __SVC_RS485_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "bsp_uart.h"

/* Exported constants --------------------------------------------------------*/
#define SVC_RS485_RX_QUEUE_LEN      8U      /* 中断 -> 调用者的接收队列 */
#define SVC_RS485_REPLY_MAX         16U     /* 单条应答的最大长度 */

#define SVC_RS485_LOCK_TIMEOUT_MS   200U    /* 等总线控制权的上限 */
#define SVC_RS485_QUERY_TIMEOUT_MS  50U     /* 等应答的默认上限 */

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  初始化 485 服务
  * @param  config 串口 / DMA / 方向脚参数，由调用方（App 层）提供。
  *                config->rx_queue 会被忽略 —— 接收队列由本服务内部创建。
  * @note   句柄通过参数传入而不是直接引用，这样本文件不需要依赖
  *         CubeMX 生成的具体符号，也让上层可以换串口。
  */
bool SvcRs485_Init(const UART_Config_t *config);

/**
  * @brief  发送一条不需要应答的命令（运动类命令用这个）
  * @note   加锁串行化，发送本身是阻塞的（等 TC 后切回接收方向）
  */
bool SvcRs485_Send(const uint8_t *frame, uint8_t len);

/**
  * @brief  发送一条需要应答的查询（读取类命令用这个）
  * @param  expect_addr 期望的应答地址，用于过滤总线上其它设备的报文
  * @param  reply       接收缓冲
  * @param  reply_len   输出实际长度
  * @param  timeout_ms  等应答的超时
  * @retval false 表示超时或应答不匹配
  */
bool SvcRs485_Query(const uint8_t *frame, uint8_t len, uint8_t expect_addr,
                    uint8_t *reply, uint16_t reply_cap, uint16_t *reply_len,
                    uint32_t timeout_ms);

/** @brief 获取底层实例，用于读取统计信息 */
const UART_Inst_t *SvcRs485_GetInst(void);

/** @brief 服务是否已初始化（未初始化时所有接口直接返回失败） */
bool SvcRs485_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __SVC_RS485_H__ */
