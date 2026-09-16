/**
  ******************************************************************************
  * @file    y42.h
  * @brief   ZDT（张大头）Y42 闭环步进电机串口协议编解码
  ******************************************************************************
  * @note    本模块是纯逻辑代码：只把参数打包成字节、把字节解析成参数，
  *          **不发送、不接收、不碰 HAL / UART**，可在 PC 上做单元测试。
  *
  * 报文格式（依据官方手册《ZDT Y42 第二代闭环步进电机使用说明 V1.1》）：
  *
  *   主机发送： | 地址 | 功能码 | 命令数据 | 校验码 |
  *   电机返回： | 地址 | 功能码 | 返回数据 | 校验码 |
  *
  *   校验码出厂默认为固定字节 0x6B，也可配置为 XOR / CRC8 / Modbus-RTU。
  *   本工程按出厂默认的 0x6B 实现。
  *
  * 控制类命令的返回数据是 1 字节状态（手册 4.1.2 节）：
  *   0x02 命令正确
  *   0xE2 参数错误 / 范围不满足 / 触发保护
  *   0xEE 命令格式错误
  *   0x9F 动作执行完成（电机主动返回）
  *
  * 读取类命令的返回数据段长度随命令而异，骨架统一是
  *   地址 + 功能码 + 返回数据(N) + 校验码
  * 例如：
  *   读实时位置 0x36 → 8 字节：地址 + 36 + 符号(1) + 位置(4,大端) + 6B
  *                     符号 0x00 = 正，0x01 = 负（不是二补码！）
  *                     实机例： 01 36 01 00 00 00 03 6B  =  位置 -3
  *   读驱动温度 0x39 → 5 字节：地址 + 39 + 符号(1) + 温度(1) + 6B
  *
  * 【重要】同一个位置数值在两种固件下含义完全不同（手册 5.5.13 节）：
  *   Emm 固件：0~65535 表示一圈 0~360°，角度 = 数值 * 360 / 65536
  *   X   固件：角度 = 数值 / 10
  *
  ******************************************************************************
  */

#ifndef __Y42_H__
#define __Y42_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
#define Y42_ADDR_BROADCAST      0x00U   /* 广播地址 */
#define Y42_ADDR_MIN            1U
#define Y42_ADDR_MAX            255U

#define Y42_CHECK_BYTE          0x6BU   /* 出厂默认校验字节 */
#define Y42_ACK_STATUS_OK       0x02U   /* 控制类应答：命令正确 */
#define Y42_ACK_STATUS_PARAM    0xE2U   /* 控制类应答：参数错误 */
#define Y42_ACK_STATUS_FORMAT   0xEEU   /* 控制类应答：格式错误 */
#define Y42_ACK_STATUS_DONE     0x9FU   /* 动作执行完成（主动返回） */

#define Y42_VEL_MAX_RPM         5000U   /* 转速上限 */
#define Y42_ACC_MAX             255U    /* 加速度上限，0 表示直接启动 */

#define Y42_FRAME_MAX           16U     /* 发送帧最大长度（位置模式 13 字节） */
#define Y42_REPLY_MAX           16U     /* 应答帧最大长度 */

/* 读取类应答的骨架：地址(1) + 功能码(1) + 返回数据(N) + 校验(1) */
#define Y42_REPLY_MIN_LEN       4U
#define Y42_REPLY_DATA_MAX      12U

/* 读实时位置(0x36)的返回数据段：符号(1) + 位置(4) */
#define Y42_POS_REPLY_DATA_LEN  5U

/* 符号字节取值 */
#define Y42_SIGN_POSITIVE       0x00U
#define Y42_SIGN_NEGATIVE       0x01U

/* 固件类型 —— 决定位置数值如何换算成角度 */
#define Y42_FW_EMM              0U      /* 0~65535 对应一圈 0~360° */
#define Y42_FW_X                1U      /* 角度 = 数值 / 10 */

/* 方向 */
#define Y42_DIR_CW              0U
#define Y42_DIR_CCW             1U

/* 位置模式的运动标志 raF */
#define Y42_MOVE_REL_LAST       0U      /* 相对上一次的目标位置 */
#define Y42_MOVE_ABSOLUTE       1U      /* 绝对位置 */
#define Y42_MOVE_REL_NOW        2U      /* 相对当前实时位置 */

/* 功能码 */
#define Y42_CODE_ENC_CAL        0x06U
#define Y42_CODE_RESET_MOTOR    0x08U
#define Y42_CODE_RESET_CURPOS   0x0AU
#define Y42_CODE_RESET_CLOG     0x0EU
#define Y42_CODE_RESTORE        0x0FU
#define Y42_CODE_SET_QPOS       0xF1U
#define Y42_CODE_EN_CONTROL     0xF3U
#define Y42_CODE_VEL_CONTROL    0xF6U
#define Y42_CODE_QPOS_CONTROL   0xFCU
#define Y42_CODE_POS_CONTROL    0xFDU
#define Y42_CODE_STOP_NOW       0xFEU
#define Y42_CODE_SYNC_MOTION    0xFFU
#define Y42_CODE_READ_STATE     0x43U
#define Y42_CODE_READ_CONF      0x42U

/* 读取系统参数时使用的子系统码 */
#define Y42_INFO_VBUS           0x24U   /* 总线电压 */
#define Y42_INFO_CBUS           0x26U   /* 总线电流 */
#define Y42_INFO_CPHA           0x27U   /* 相电流 */
#define Y42_INFO_ENCO           0x29U   /* 编码器原始值 */
#define Y42_INFO_CLKC           0x30U   /* 实时脉冲数 */
#define Y42_INFO_ENCL           0x31U   /* 线性化后的编码器值 */
#define Y42_INFO_CLKI           0x32U   /* 输入脉冲数 */
#define Y42_INFO_TPOS           0x33U   /* 目标位置 */
#define Y42_INFO_SPOS           0x34U   /* 实时设定的目标位置 */
#define Y42_INFO_VEL            0x35U   /* 实时转速 */
#define Y42_INFO_CPOS           0x36U   /* 实时位置 */
#define Y42_INFO_PERR           0x37U   /* 位置误差 */
#define Y42_INFO_VBAT           0x38U   /* 多圈编码器电池电压 */
#define Y42_INFO_TEMP           0x39U   /* 实时温度 */
#define Y42_INFO_FLAG           0x3AU   /* 电机状态标志位 */
#define Y42_INFO_OFLAG          0x3BU   /* 回零状态标志位 */
#define Y42_INFO_OAF            0x3CU   /* 状态标志 + 回零标志 */
#define Y42_INFO_PIN            0x3DU   /* 引脚状态 */

/*
 * 【两个量纲不要混】
 *   位置模式"命令"的脉冲数：默认 1.8° 电机 + 16 细分 = 3200 脉冲/圈
 *                             （手册 5.3.12，与固件类型无关，取决于细分设置）
 *   读取"实时位置"的返回值：Emm 固件 65536 计数/圈；X 固件 0.1°/计数
 *                             （手册 5.5.13，随固件类型而变）
 */
#define Y42_CMD_PULSE_PER_REV   3200U

/* Exported types ------------------------------------------------------------*/
/** 一条应答的内容 */
typedef struct
{
  uint8_t addr;
  uint8_t code;
  uint8_t data[Y42_REPLY_DATA_MAX];   /* 功能码与校验码之间的原始字节 */
  uint8_t data_len;
} Y42_Reply_t;

/* Exported functions prototypes ---------------------------------------------*/

/* ---- 运动控制类：均返回打包后的帧长度，0 表示参数非法 ---- */
uint8_t Y42_BuildEnControl(uint8_t *out, uint8_t addr, bool enable, bool sync);

uint8_t Y42_BuildVelControl(uint8_t *out, uint8_t addr,
                            uint8_t dir, uint16_t vel_rpm,
                            uint8_t acc, bool sync);

uint8_t Y42_BuildPosControl(uint8_t *out, uint8_t addr,
                            uint8_t dir, uint16_t vel_rpm, uint8_t acc,
                            uint32_t clk, uint8_t move_mode, bool sync);

uint8_t Y42_BuildStopNow(uint8_t *out, uint8_t addr, bool sync);
uint8_t Y42_BuildSyncMotion(uint8_t *out, uint8_t addr);
uint8_t Y42_BuildResetCurPos(uint8_t *out, uint8_t addr);

/* ---- 读取类 ---- */
uint8_t Y42_BuildReadSysParam(uint8_t *out, uint8_t addr, uint8_t info_code);
uint8_t Y42_BuildReadState(uint8_t *out, uint8_t addr);
uint8_t Y42_BuildReadConf(uint8_t *out, uint8_t addr);

/* ---- 应答解析 ---- */

/** @brief 控制类应答校验（4 字节：地址 + 功能码 + 状态 + 校验码） */
bool Y42_ParseAck(const uint8_t *buf, uint16_t len,
                  uint8_t expect_addr, uint8_t expect_code);

/** @brief 读取类应答校验：地址匹配、末字节为校验码，剥离出返回数据段 */
bool Y42_ParseReply(const uint8_t *buf, uint16_t len,
                    uint8_t expect_addr, Y42_Reply_t *reply);

/** @brief 从 0x36 的应答里解出带符号的实时位置 */
bool Y42_DecodePosition(const Y42_Reply_t *reply, int32_t *position);

/** @brief 把位置原始值换算成角度（度），固件类型影响换算公式 */
float Y42_PositionToDegrees(int32_t position, uint8_t firmware);

/** @brief 把角度（度）换算成位置模式命令的脉冲数（3200 脉冲/圈） */
uint32_t Y42_DegreesToPulses(float degrees);

#ifdef __cplusplus
}
#endif

#endif /* __Y42_H__ */
