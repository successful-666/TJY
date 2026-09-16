/**
  ******************************************************************************
  * @file    log.c
  * @brief   线程安全的日志输出
  ******************************************************************************
  * @note    本文件由手工维护，不属于 STM32CubeMX 生成范围。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "log.h"

#include <stdio.h>
#include <stdarg.h>

#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
/* 保护 printf 的互斥锁，NULL 表示尚未初始化 */
static osMutexId_t logMutex = NULL;

/* Exported functions --------------------------------------------------------*/
void log_init(void)
{
  if (logMutex == NULL)
  {
    logMutex = osMutexNew(NULL);
  }
}

void log_printf(const char *tag, const char *fmt, ...)
{
  va_list ap;

  /* 锁的作用：保证一条日志内部的多次 printf 不会被别的任务插进来 */
  if (logMutex != NULL)
  {
    (void)osMutexAcquire(logMutex, osWaitForever);
  }

  printf("[%s] ", (tag != NULL) ? tag : "-");

  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);

  printf("\r\n");

  if (logMutex != NULL)
  {
    (void)osMutexRelease(logMutex);
  }
}
