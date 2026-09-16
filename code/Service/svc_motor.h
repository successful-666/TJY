/**
  ******************************************************************************
  * @file    svc_motor.h
  * @brief   电机业务层：管理 DM3519 的状态缓存、控制指令与在线检测
  ******************************************************************************
  * @note    本层只调用 CAN 服务的接口，不直接接触 HAL，也不解析字节。
  ******************************************************************************
  */

#ifndef __SVC_MOTOR_H__
#define __SVC_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "dm3519.h"
#include "y42.h"

/* Exported constants --------------------------------------------------------*/
#define SVC_MOTOR_DM_COUNT         DM3519_COUNT

#define SVC_MOTOR_DM_OFFLINE_MS    1000U   /* 超过该时间无反馈判定离线 */
#define SVC_MOTOR_MONITOR_MS       100U    /* 在线检测任务周期 */

/* Y42 的固件类型 —— 决定"读取回来的位置值"如何换算成角度。
   已确认为 Emm 固件：65536 计数对应一圈 360°。
   （X 固件的换算不同：角度 = 数值 / 10。）
   注意这个宏只影响"读取"；位置模式命令的脉冲数固定 3200/圈。 */
#define SVC_MOTOR_Y42_FIRMWARE     Y42_FW_EMM

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化电机业务层并启动在线检测任务 */
bool SvcMotor_Init(void);

/** @brief CAN 服务收到帧时的回调，由 SvcCan_Init 注册 */
void SvcMotor_OnCanFrame(const CAN_Frame_t *frame);

/* ---- DM3519 控制接口（id 范围 1 ~ SVC_MOTOR_DM_COUNT） ---- */

/** @brief 使能 / 失能单个电机 */
bool SvcMotor_DmEnable(uint8_t id, bool enable);

/** @brief 使能 / 失能全部电机 */
bool SvcMotor_DmEnableAll(bool enable);

/** @brief 设定速度（rpm，正负代表方向） */
bool SvcMotor_DmSetSpeedRpm(uint8_t id, int16_t speed_rpm);

/** @brief 全部电机速度归零（先限速到 0，再失能） */
bool SvcMotor_DmStopAll(void);

/** @brief 读取某个电机的状态快照，返回 NULL 表示 id 非法 */
const DM3519_t *SvcMotor_GetDm(uint8_t id);

/* ---- Y42（485）控制接口 ---- */

/**
  * @brief  使能 / 失能某个 Y42
  * @note   运动类命令不等应答，发出去即返回
  */
bool SvcMotor_Y42Enable(uint8_t addr, bool enable);

/** @brief 立即停止某个 Y42 */
bool SvcMotor_Y42Stop(uint8_t addr);

/** @brief 速度模式：dir = Y42_DIR_CW / Y42_DIR_CCW，vel_rpm 为幅值 */
bool SvcMotor_Y42SetSpeedRpm(uint8_t addr, uint8_t dir,
                             uint16_t vel_rpm, uint8_t acc);

/** @brief 将 Y42 当前位置清零 */
bool SvcMotor_Y42ResetCurPos(uint8_t addr);

/**
  * @brief  位置模式：走到指定脉冲数
  * @param  move_mode Y42_MOVE_ABSOLUTE（绝对）/ Y42_MOVE_REL_NOW（相对当前位置）
  * @param  vel_rpm   转速，1 ~ Y42_VEL_MAX_RPM
  * @param  acc       加速度，0 表示直接启动
  */
bool SvcMotor_Y42SetPosPulses(uint8_t addr, int32_t pulses,
                              uint16_t vel_rpm, uint8_t acc,
                              uint8_t move_mode);

/**
  * @brief  位置模式：走到指定角度（按 3200 脉冲/圈换算）
  * @note   方向由角度正负自动决定
  */
bool SvcMotor_Y42SetPosDegrees(uint8_t addr, float degrees,
                               uint16_t vel_rpm, uint8_t acc,
                               uint8_t move_mode);

/**
  * @brief  相对当前位置走一个角度（正负表示方向）
  * @note   相对模式不依赖零点参考，适合标定和试转
  */
bool SvcMotor_Y42MoveDegrees(uint8_t addr, float degrees,
                             uint16_t vel_rpm, uint8_t acc);

/**
  * @brief  读取 Y42 实时位置（脉冲）
  * @retval false 表示超时或应答非法
  */
bool SvcMotor_Y42GetPosPulses(uint8_t addr, int32_t *pulses);

/** @brief 读取 Y42 实时位置（角度） */
bool SvcMotor_Y42GetPosDegrees(uint8_t addr, float *degrees);

#ifdef __cplusplus
}
#endif

#endif /* __SVC_MOTOR_H__ */
