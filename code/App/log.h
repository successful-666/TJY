/**
  ******************************************************************************
  * @file    log.h
  * @brief   线程安全的日志输出（底层走 printf -> USART1）
  ******************************************************************************
  * @note    本文件由手工维护，不属于 STM32CubeMX 生成范围。
  ******************************************************************************
  */

#ifndef __LOG_H__
#define __LOG_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  初始化日志模块，创建内部互斥锁
  * @note   必须在 FreeRTOS 调度器启动之后调用一次
  */
void log_init(void);

/**
  * @brief  输出一条日志，自动加标签和换行，内部串行化
  * @param  tag: 模块标签，例如 "MAIN" / "ETH" / "CAN" / "485"
  * @param  fmt: 格式串，用法同 printf
  * @note   禁止在中断服务函数里调用
  */
void log_printf(const char *tag, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __LOG_H__ */
