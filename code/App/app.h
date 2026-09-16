/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层入口
  ******************************************************************************
  * @note    本文件由手工维护，不属于 STM32CubeMX 生成范围。
  *          App_Init() 是整个手写代码树唯一的启动入口，
  *          由 Core/Src/freertos.c 的 StartDefaultTask 调用。
  ******************************************************************************
  */

#ifndef __APP_H__
#define __APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  初始化整个应用：电机业务 -> CAN 服务 -> TCP 服务
  * @note   必须在 FreeRTOS 调度器启动后调用
  */
void App_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H__ */
