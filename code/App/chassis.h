/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   底盘运动学：把「前进速度 + 转向角速度」分解成各轮转速与转向角
  ******************************************************************************
  * @note    本模块是纯逻辑代码，不依赖 HAL / FreeRTOS / 任何电机接口，
  *          可以在 PC 上直接做单元测试。
  *
  *          移植自参考工程 Frame_work/APP/app.c 中的底盘控制部分，
  *          区别是转向执行器由「总线舵机」换成了「Y42 步进电机」，
  *          但运动学部分（差速分解 + 限幅 + 方向系数）完全保留。
  ******************************************************************************
  */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
#define CHASSIS_WHEEL_COUNT     4U
#define CHASSIS_STEER_COUNT     2U

/* 轮子索引：与参考工程一致 —— 左前、右前、左后、右后 */
#define CHASSIS_WHEEL_FL        0U
#define CHASSIS_WHEEL_FR        1U
#define CHASSIS_WHEEL_RL        2U
#define CHASSIS_WHEEL_RR        3U

/* 转向索引 */
#define CHASSIS_STEER_LEFT      0U
#define CHASSIS_STEER_RIGHT     1U

/* Exported types ------------------------------------------------------------*/
/** 底盘配置。方向系数和基准角必须在实车上标定后填写 */
typedef struct
{
  /* ---- 驱动轮 ---- */
  uint8_t wheel_dm_id[CHASSIS_WHEEL_COUNT];  /* 每个轮子对应的 DM3519 ID */
  int8_t  wheel_dir[CHASSIS_WHEEL_COUNT];    /* 安装方向：+1 / -1 */
  int16_t max_wheel_rpm;                     /* 单轮转速上限 */

  /* ---- 转向 ---- */
  uint8_t steer_y42_addr[CHASSIS_STEER_COUNT];  /* Y42 的 485 地址 */
  int8_t  steer_dir[CHASSIS_STEER_COUNT];       /* 镜像安装方向系数 */
  int16_t steer_ref_deg[CHASSIS_STEER_COUNT];   /* 实测直行基准角（度） */
  int16_t steer_max_offset_deg;                 /* 相对基准角的最大转角 */
  uint16_t steer_vel_rpm;                       /* 转向电机转速 */
  uint8_t  steer_acc;                           /* 转向电机加速度，0=直接启动 */
} Chassis_Config_t;

/** 一次运动解算的结果 */
typedef struct
{
  /* 各轮最终发给电机的转速，**已经乘过方向系数**。
     左右轮镜像安装时，物理同向的两个轮子在这里的符号是相反的。 */
  int16_t wheel_rpm[CHASSIS_WHEEL_COUNT];

  /* 各转向机构的绝对目标角度（度） */
  int16_t steer_angle_deg[CHASSIS_STEER_COUNT];
} Chassis_Output_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 设置底盘配置（实车标定值）。未调用则使用内部默认值 */
void Chassis_Init(const Chassis_Config_t *config);

/** @brief 取当前配置，只读 */
const Chassis_Config_t *Chassis_GetConfig(void);

/** @brief 单轮转速对称限幅 */
int16_t Chassis_LimitRpm(int16_t rpm);

/**
  * @brief  运动解算：差速分解 + 限幅 + 方向系数 + 转向角计算
  * @param  linear_rpm       前进/后退速度，正为前进
  * @param  yaw_rpm          转向角速度，正为左转（左轮后退、右轮前进）
  * @param  steer_offset_deg 转向偏移角，0 = 回正，负=左转，正=右转
  * @param  out              解算结果
  * @retval false 表示参数非法或底盘未初始化
  */
bool Chassis_Mix(int16_t linear_rpm, int16_t yaw_rpm,
                 int16_t steer_offset_deg, Chassis_Output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H__ */
