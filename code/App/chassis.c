/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   底盘运动学实现
  ******************************************************************************
  * @note    算法来源：参考工程 Frame_work/APP/app.c 的
  *          App_LimitWheelRPM / App_ChassisSetMotion / App_SteeringSetOffset，
  *          数值与符号约定保持一致。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "chassis.h"

#include <stddef.h>

/* Private define ------------------------------------------------------------*/
/* 默认配置：方向系数沿用参考工程的实测值（左右镜像，符号相反）。
   注意 steer_ref_deg 是【占位值】，必须在实车上标定后通过 Chassis_Init 覆盖。 */
#define CHASSIS_DEFAULT_MAX_RPM        20
#define CHASSIS_DEFAULT_STEER_MAX_OFF  45
#define CHASSIS_DEFAULT_STEER_VEL_RPM  300
#define CHASSIS_DEFAULT_STEER_ACC      10

/* Private variables ---------------------------------------------------------*/
static Chassis_Config_t s_cfg;
static bool             s_ready = false;

/* Private functions ---------------------------------------------------------*/
static int16_t ClampInt16(int32_t value, int16_t limit)
{
  if (value > (int32_t)limit)  { return limit; }
  if (value < -(int32_t)limit) { return (int16_t)(-limit); }
  return (int16_t)value;
}

/* Exported functions --------------------------------------------------------*/
void Chassis_Init(const Chassis_Config_t *config)
{
  if (config != NULL)
  {
    s_cfg = *config;
  }
  else
  {
    /* 无参数时用一套安全的默认值，避免解算出垃圾数据 */
    s_cfg.wheel_dm_id[CHASSIS_WHEEL_FL] = 1U;
    s_cfg.wheel_dm_id[CHASSIS_WHEEL_FR] = 2U;
    s_cfg.wheel_dm_id[CHASSIS_WHEEL_RL] = 3U;
    s_cfg.wheel_dm_id[CHASSIS_WHEEL_RR] = 4U;

    s_cfg.wheel_dir[CHASSIS_WHEEL_FL] =  1;
    s_cfg.wheel_dir[CHASSIS_WHEEL_FR] = -1;
    s_cfg.wheel_dir[CHASSIS_WHEEL_RL] =  1;
    s_cfg.wheel_dir[CHASSIS_WHEEL_RR] = -1;

    s_cfg.max_wheel_rpm = CHASSIS_DEFAULT_MAX_RPM;

    s_cfg.steer_y42_addr[CHASSIS_STEER_LEFT]  = 1U;
    s_cfg.steer_y42_addr[CHASSIS_STEER_RIGHT] = 2U;
    s_cfg.steer_dir[CHASSIS_STEER_LEFT]       =  1;
    s_cfg.steer_dir[CHASSIS_STEER_RIGHT]      = -1;

    /* 占位基准角，实车标定后必须覆盖 */
    s_cfg.steer_ref_deg[CHASSIS_STEER_LEFT]  = 0;
    s_cfg.steer_ref_deg[CHASSIS_STEER_RIGHT] = 0;

    s_cfg.steer_max_offset_deg = CHASSIS_DEFAULT_STEER_MAX_OFF;
    s_cfg.steer_vel_rpm        = CHASSIS_DEFAULT_STEER_VEL_RPM;
    s_cfg.steer_acc            = CHASSIS_DEFAULT_STEER_ACC;
  }

  s_ready = true;
}

const Chassis_Config_t *Chassis_GetConfig(void)
{
  return s_ready ? &s_cfg : NULL;
}

int16_t Chassis_LimitRpm(int16_t rpm)
{
  return ClampInt16((int32_t)rpm, s_cfg.max_wheel_rpm);
}

bool Chassis_Mix(int16_t linear_rpm, int16_t yaw_rpm,
                 int16_t steer_offset_deg, Chassis_Output_t *out)
{
  int16_t left_rpm;
  int16_t right_rpm;
  int16_t offset;
  uint8_t i;

  if ((out == NULL) || (!s_ready) || (s_cfg.max_wheel_rpm <= 0))
  {
    return false;
  }

  /* ---- 第一步：差速分解 ----
     直行时 linear 同时加到两侧；原地转向时 linear 为 0，两侧反向。
     先分解再限幅 —— 这样打舵时不会因为单侧超速而丢失转向比例。 */
  left_rpm  = Chassis_LimitRpm((int16_t)(linear_rpm - yaw_rpm));
  right_rpm = Chassis_LimitRpm((int16_t)(linear_rpm + yaw_rpm));

  /* ---- 第二步：分配到四个轮子并乘方向系数 ----
     左右两侧镜像安装，物理同向的两个轮子电信号相反。
     所以这里输出的是"电机坐标系"下的值，不是物理转速。 */
  out->wheel_rpm[CHASSIS_WHEEL_FL] = (int16_t)(left_rpm  * s_cfg.wheel_dir[CHASSIS_WHEEL_FL]);
  out->wheel_rpm[CHASSIS_WHEEL_RL] = (int16_t)(left_rpm  * s_cfg.wheel_dir[CHASSIS_WHEEL_RL]);
  out->wheel_rpm[CHASSIS_WHEEL_FR] = (int16_t)(right_rpm * s_cfg.wheel_dir[CHASSIS_WHEEL_FR]);
  out->wheel_rpm[CHASSIS_WHEEL_RR] = (int16_t)(right_rpm * s_cfg.wheel_dir[CHASSIS_WHEEL_RR]);

  /* ---- 第三步：转向角 ----
     在实测基准角上叠加偏移量，两台转向机构镜像（一台加、一台减）。
     偏移量先限幅，避免打过头把机械结构顶死。 */
  offset = ClampInt16((int32_t)steer_offset_deg, s_cfg.steer_max_offset_deg);

  for (i = 0U; i < CHASSIS_STEER_COUNT; i++)
  {
    out->steer_angle_deg[i] =
        (int16_t)(s_cfg.steer_ref_deg[i] + offset * s_cfg.steer_dir[i]);
  }

  return true;
}
