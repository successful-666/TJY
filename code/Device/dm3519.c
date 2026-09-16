/**
  ******************************************************************************
  * @file    dm3519.c
  * @brief   达妙 DM3519 电机 CAN 协议编解码实现
  ******************************************************************************
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dm3519.h"

#include <string.h>

/* Private define ------------------------------------------------------------*/
#define DM3519_PI                3.14159265358979323846f
#define DM3519_FRAME_DLC         8U
#define DM3519_RAW_MAX           2047.0f
#define DM3519_POS_RAW_MAX       65535.0f

/* Private functions ---------------------------------------------------------*/
/** 把反馈里的 12 位二进制补码扩展成 int16_t */
static int16_t SignExtend12(uint16_t value)
{
  return ((value & 0x0800U) != 0U) ? (int16_t)(value | 0xF000U)
                                   : (int16_t)value;
}

/* Exported functions --------------------------------------------------------*/
void DM3519_Init(DM3519_t *motor, uint8_t id)
{
  if (motor == NULL)
  {
    return;
  }

  memset(motor, 0, sizeof(*motor));
  motor->id = id;
}

bool DM3519_BuildEnableFrame(const DM3519_t *motor,
                             bool enable,
                             CAN_Frame_t *frame)
{
  if ((motor == NULL) || (frame == NULL) ||
      (motor->id == 0U) || (motor->id > DM3519_COUNT))
  {
    return false;
  }

  memset(frame, 0, sizeof(*frame));
  frame->id  = DM3519_ID_ENABLE_BASE + motor->id;
  frame->ide = CAN_ID_STD;
  frame->rtr = CAN_RTR_DATA;
  frame->dlc = DM3519_FRAME_DLC;

  memset(frame->data, 0xFF, 7U);
  frame->data[7] = enable ? DM3519_SPECIAL_BYTE_ENABLE
                          : DM3519_SPECIAL_BYTE_DISABLE;
  return true;
}

bool DM3519_BuildSpeedFrame(DM3519_t *motor,
                            int16_t speed_rpm,
                            CAN_Frame_t *frame)
{
  float speed_rad_s;

  if ((motor == NULL) || (frame == NULL) ||
      (motor->id == 0U) || (motor->id > DM3519_COUNT))
  {
    return false;
  }

  /* 速度模式要求前 4 字节为 IEEE754 rad/s，小端 MCU 可直接 memcpy */
  speed_rad_s = (float)speed_rpm * (2.0f * DM3519_PI) / 60.0f;

  memset(frame, 0, sizeof(*frame));
  frame->id  = DM3519_ID_VELOCITY_BASE + motor->id;
  frame->ide = CAN_ID_STD;
  frame->rtr = CAN_RTR_DATA;
  frame->dlc = DM3519_FRAME_DLC;
  memcpy(frame->data, &speed_rad_s, sizeof(speed_rad_s));

  motor->target_speed_rpm = speed_rpm;
  return true;
}

bool DM3519_ParseFrame(DM3519_t *motor,
                       const CAN_Frame_t *frame,
                       uint32_t now_ms)
{
  uint16_t position_raw;
  uint16_t velocity_u12;
  uint16_t torque_u12;
  uint8_t  embedded_id;

  if ((motor == NULL) || (frame == NULL))
  {
    return false;
  }
  if ((frame->ide != CAN_ID_STD) || (frame->rtr != CAN_RTR_DATA) ||
      (frame->dlc != DM3519_FRAME_DLC))
  {
    return false;
  }
  if ((motor->id == 0U) || (motor->id > DM3519_COUNT))
  {
    return false;
  }

  /* 同时兼容"用 CAN ID 区分反馈"和"数据区携带电机 ID"两种配置 */
  embedded_id = frame->data[1] & 0x0FU;
  if ((frame->id != (DM3519_ID_FEEDBACK_BASE + motor->id)) &&
      (embedded_id != motor->id))
  {
    return false;
  }

  motor->error_code = (uint8_t)((frame->data[1] >> 4) & 0x0FU);
  motor->enabled    = (motor->error_code == 1U);

  /* 位置：16 位线性映射到 [-POS_MAX, +POS_MAX] */
  position_raw = ((uint16_t)frame->data[2] << 8) | (uint16_t)frame->data[3];
  motor->position_rad = ((float)position_raw / DM3519_POS_RAW_MAX) *
                        (2.0f * DM3519_POS_MAX_RAD) - DM3519_POS_MAX_RAD;

  /* 速度：跨字节打包的 12 位有符号数 */
  velocity_u12 = ((uint16_t)frame->data[4] << 4) |
                 ((uint16_t)frame->data[5] & 0x0FU);
  motor->velocity_raw   = SignExtend12(velocity_u12);
  motor->velocity_rad_s = (float)motor->velocity_raw *
                          DM3519_VEL_MAX_RAD_S / DM3519_RAW_MAX;
  motor->velocity_rpm   = motor->velocity_rad_s * 60.0f / (2.0f * DM3519_PI);

  /* 力矩：同样 12 位打包 */
  torque_u12 = ((uint16_t)(frame->data[5] & 0xF0U) << 4) |
               (uint16_t)frame->data[6];
  motor->torque_raw = SignExtend12(torque_u12);
  motor->torque_nm  = (float)motor->torque_raw *
                      DM3519_TORQUE_MAX_NM / DM3519_RAW_MAX;

  motor->mos_temperature   = (uint8_t)((frame->data[7] >> 4) & 0x0FU);
  motor->rotor_temperature = (uint8_t)(frame->data[7] & 0x0FU);

  motor->last_rx_tick = now_ms;
  motor->online       = true;
  motor->rx_count++;

  return true;
}

void DM3519_UpdateOnline(DM3519_t *motor,
                         uint32_t now_ms,
                         uint32_t timeout_ms)
{
  if (motor == NULL)
  {
    return;
  }

  /* 无符号相减可正确处理 tick 回绕 */
  if (motor->online &&
      ((uint32_t)(now_ms - motor->last_rx_tick) > timeout_ms))
  {
    motor->online = false;
  }
}
