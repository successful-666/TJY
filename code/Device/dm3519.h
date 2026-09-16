/**
  ******************************************************************************
  * @file    dm3519.h
  * @brief   达妙 DM3519 电机 CAN 协议编解码
  ******************************************************************************
  * @note    本模块是纯逻辑代码：只做帧的编解码，不发送、不接收、不碰 HAL。
  *          因此可以脱离硬件在 PC 上做单元测试。
  ******************************************************************************
  */

#ifndef __DM3519_H__
#define __DM3519_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "bsp_can.h"

/* Exported constants --------------------------------------------------------*/
#define DM3519_COUNT            4U

#define DM3519_VEL_MAX_RAD_S    30.0f
#define DM3519_TORQUE_MAX_NM    7.8f
#define DM3519_POS_MAX_RAD      12.5f

/* 特殊控制帧：前 7 字节固定 0xFF，末字节 0xFC 使能 / 0xFD 失能 */
#define DM3519_SPECIAL_BYTE_ENABLE   0xFCU
#define DM3519_SPECIAL_BYTE_DISABLE  0xFDU

/* 命令 ID 基址（标准帧 11 位） */
#define DM3519_ID_ENABLE_BASE   0x100U
#define DM3519_ID_VELOCITY_BASE 0x200U
#define DM3519_ID_FEEDBACK_BASE 0x200U

/* Exported types ------------------------------------------------------------*/
/** 单个 DM3519 的配置、反馈状态和最后一次控制目标 */
typedef struct
{
  uint8_t  id;                     /* 1 ~ DM3519_COUNT */
  bool     enabled;                /* 反馈中解析出的使能状态 */
  bool     online;                 /* 最近是否收到过反馈 */
  uint8_t  error_code;

  int16_t  velocity_raw;
  int16_t  torque_raw;
  float    position_rad;
  float    velocity_rad_s;
  float    velocity_rpm;
  float    torque_nm;
  uint8_t  mos_temperature;
  uint8_t  rotor_temperature;

  int16_t  target_speed_rpm;       /* 最后一次下发的目标，便于回读 */
  uint32_t last_rx_tick;
  uint32_t rx_count;
  uint32_t parse_error_count;
} DM3519_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化一个电机对象，id 范围为 1 ~ DM3519_COUNT */
void DM3519_Init(DM3519_t *motor, uint8_t id);

/** @brief 生成使能/失能帧，不直接访问 CAN 外设 */
bool DM3519_BuildEnableFrame(const DM3519_t *motor,
                             bool enable,
                             CAN_Frame_t *frame);

/** @brief 生成速度模式帧（float32 rad/s，小端） */
bool DM3519_BuildSpeedFrame(DM3519_t *motor,
                            int16_t speed_rpm,
                            CAN_Frame_t *frame);

/** @brief 尝试解析一帧反馈；帧不属于该电机时返回 false */
bool DM3519_ParseFrame(DM3519_t *motor,
                       const CAN_Frame_t *frame,
                       uint32_t now_ms);

/** @brief 超时未收到反馈则标记为离线 */
void DM3519_UpdateOnline(DM3519_t *motor,
                         uint32_t now_ms,
                         uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __DM3519_H__ */
