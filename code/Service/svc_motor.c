/**
  ******************************************************************************
  * @file    svc_motor.c
  * @brief   电机业务层实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "svc_motor.h"
#include "svc_can.h"
#include "svc_rs485.h"
#include "log.h"

/* Private variables ---------------------------------------------------------*/
static DM3519_t     s_dm[SVC_MOTOR_DM_COUNT + 1U];   /* 下标 1 起用，便于按 ID 访问 */
static bool         s_prev_online[SVC_MOTOR_DM_COUNT + 1U];
static uint8_t      s_prev_error[SVC_MOTOR_DM_COUNT + 1U];
static osThreadId_t s_monitor_task = NULL;
static bool         s_initialized = false;

/* Private function prototypes -----------------------------------------------*/
static void SvcMotor_MonitorTask(void *argument);

static const osThreadAttr_t s_monitor_task_attr = {
  .name       = "MotorMon",
  .stack_size = 1024U,
  .priority   = (osPriority_t)osPriorityBelowNormal,
};

/* Private functions ---------------------------------------------------------*/
static bool IsValidId(uint8_t id)
{
  return ((id >= 1U) && (id <= SVC_MOTOR_DM_COUNT));
}

/**
  * @brief 把一段字节以十六进制打进日志，用于确认设备应答的真实格式
  * @note  手写十六进制转换而不是用 printf 的 %X，避免依赖 MicroLib 的
  *        格式化能力（尤其是 %s 之外的高级格式）。
  */
static void LogHexFrame(uint8_t addr, const uint8_t *buf, uint16_t len)
{
  static const char hexdig[] = "0123456789ABCDEF";
  char     out[3U * SVC_RS485_REPLY_MAX + 1U];
  uint16_t i;
  uint16_t n = 0U;

  if (len > SVC_RS485_REPLY_MAX)
  {
    len = SVC_RS485_REPLY_MAX;
  }

  for (i = 0U; i < len; i++)
  {
    out[n] = hexdig[(buf[i] >> 4) & 0x0FU]; n++;
    out[n] = hexdig[buf[i] & 0x0FU];        n++;
    out[n] = ' ';                           n++;
  }
  out[n] = '\0';

  log_printf("MOT", "Y42[%u] reply len=%u : %s",
             (unsigned)addr, (unsigned)len, out);
}

/* Exported functions --------------------------------------------------------*/
bool SvcMotor_Init(void)
{
  uint8_t i;

  for (i = 1U; i <= SVC_MOTOR_DM_COUNT; i++)
  {
    DM3519_Init(&s_dm[i], i);
    s_prev_online[i] = false;
    s_prev_error[i]  = 0U;
  }

  s_monitor_task = osThreadNew(SvcMotor_MonitorTask, NULL, &s_monitor_task_attr);
  if (s_monitor_task == NULL)
  {
    log_printf("MOT", "monitor task create failed");
    return false;
  }

  s_initialized = true;
  log_printf("MOT", "service up, %u x DM3519", (unsigned)SVC_MOTOR_DM_COUNT);
  return true;
}

void SvcMotor_OnCanFrame(const CAN_Frame_t *frame)
{
  uint8_t i;

  if ((frame == NULL) || (!s_initialized))
  {
    return;
  }

  /* 依次尝试匹配每台电机；DM3519_ParseFrame 自带归属判断 */
  for (i = 1U; i <= SVC_MOTOR_DM_COUNT; i++)
  {
    if (DM3519_ParseFrame(&s_dm[i], frame, HAL_GetTick()))
    {
      /* 只在状态跳变时打印，避免每帧都刷屏把串口堵死 */
      if (!s_prev_online[i])
      {
        s_prev_online[i] = true;
        log_printf("MOT", "DM%u online", (unsigned)i);
      }

      if (s_prev_error[i] != s_dm[i].error_code)
      {
        s_prev_error[i] = s_dm[i].error_code;
        /* 位置用毫弧度整数打印：MicroLib 的 printf 对 %f 支持不完全，
           整数换算能保证输出一定正确。 */
        log_printf("MOT", "DM%u error_code=%u pos=%ld mrad",
                   (unsigned)i, (unsigned)s_dm[i].error_code,
                   (long)(s_dm[i].position_rad * 1000.0f));
      }

      return;
    }
  }
}

bool SvcMotor_DmEnable(uint8_t id, bool enable)
{
  CAN_Frame_t frame;

  if ((!IsValidId(id)) || (!s_initialized))
  {
    return false;
  }
  if (!DM3519_BuildEnableFrame(&s_dm[id], enable, &frame))
  {
    return false;
  }
  if (!SvcCan_Send(&frame))
  {
    log_printf("MOT", "enable queue full, drop id=%u", (unsigned)id);
    return false;
  }

  log_printf("MOT", "DM%u %s", (unsigned)id, enable ? "enable" : "disable");
  return true;
}

bool SvcMotor_DmEnableAll(bool enable)
{
  bool ok = true;
  uint8_t i;

  for (i = 1U; i <= SVC_MOTOR_DM_COUNT; i++)
  {
    if (!SvcMotor_DmEnable(i, enable))
    {
      ok = false;
    }
  }

  return ok;
}

bool SvcMotor_DmSetSpeedRpm(uint8_t id, int16_t speed_rpm)
{
  CAN_Frame_t frame;

  if ((!IsValidId(id)) || (!s_initialized))
  {
    return false;
  }
  if (!DM3519_BuildSpeedFrame(&s_dm[id], speed_rpm, &frame))
  {
    return false;
  }
  if (!SvcCan_Send(&frame))
  {
    log_printf("MOT", "speed queue full, drop id=%u", (unsigned)id);
    return false;
  }

  return true;
}

bool SvcMotor_DmStopAll(void)
{
  bool ok = true;
  uint8_t i;

  for (i = 1U; i <= SVC_MOTOR_DM_COUNT; i++)
  {
    if (!SvcMotor_DmSetSpeedRpm(i, 0))
    {
      ok = false;
    }
  }

  return ok;
}

const DM3519_t *SvcMotor_GetDm(uint8_t id)
{
  if (!IsValidId(id))
  {
    return NULL;
  }

  return &s_dm[id];
}

/* ---------------- Y42（485 总线） ---------------- */
bool SvcMotor_Y42Enable(uint8_t addr, bool enable)
{
  uint8_t frame[Y42_FRAME_MAX];
  uint8_t len;

  len = Y42_BuildEnControl(frame, addr, enable, false);
  if (len == 0U)
  {
    return false;
  }

  if (!SvcRs485_Send(frame, len))
  {
    return false;
  }

  log_printf("MOT", "Y42%u %s", (unsigned)addr, enable ? "enable" : "disable");
  return true;
}

bool SvcMotor_Y42Stop(uint8_t addr)
{
  uint8_t frame[Y42_FRAME_MAX];
  uint8_t len;

  len = Y42_BuildStopNow(frame, addr, false);

  return ((len != 0U) && SvcRs485_Send(frame, len));
}

bool SvcMotor_Y42SetSpeedRpm(uint8_t addr, uint8_t dir,
                             uint16_t vel_rpm, uint8_t acc)
{
  uint8_t frame[Y42_FRAME_MAX];
  uint8_t len;

  len = Y42_BuildVelControl(frame, addr, dir, vel_rpm, acc, false);
  if (len == 0U)
  {
    return false;
  }

  return SvcRs485_Send(frame, len);
}

bool SvcMotor_Y42ResetCurPos(uint8_t addr)
{
  uint8_t frame[Y42_FRAME_MAX];
  uint8_t len;

  len = Y42_BuildResetCurPos(frame, addr);

  return ((len != 0U) && SvcRs485_Send(frame, len));
}

bool SvcMotor_Y42SetPosPulses(uint8_t addr, int32_t pulses,
                              uint16_t vel_rpm, uint8_t acc,
                              uint8_t move_mode)
{
  uint8_t frame[Y42_FRAME_MAX];
  uint8_t len;
  uint8_t dir;
  uint32_t magnitude;

  /* 位置模式的方向和脉冲数是分开的两个字段：
     负数取绝对值并标记为 CCW，正数直接标记为 CW。 */
  if (pulses < 0)
  {
    dir       = Y42_DIR_CCW;
    magnitude = (uint32_t)(-(int64_t)pulses);
  }
  else
  {
    dir       = Y42_DIR_CW;
    magnitude = (uint32_t)pulses;
  }

  len = Y42_BuildPosControl(frame, addr, dir, vel_rpm, acc,
                            magnitude, move_mode, false);
  if (len == 0U)
  {
    return false;
  }

  if (!SvcRs485_Send(frame, len))
  {
    return false;
  }

  log_printf("MOT", "Y42%u move %ld pulse @%u rpm", (unsigned)addr,
             (long)pulses, (unsigned)vel_rpm);
  return true;
}

bool SvcMotor_Y42SetPosDegrees(uint8_t addr, float degrees,
                               uint16_t vel_rpm, uint8_t acc,
                               uint8_t move_mode)
{
  return SvcMotor_Y42SetPosPulses(addr,
                                  (int32_t)Y42_DegreesToPulses(degrees),
                                  vel_rpm, acc, move_mode);
}

bool SvcMotor_Y42MoveDegrees(uint8_t addr, float degrees,
                             uint16_t vel_rpm, uint8_t acc)
{
  int32_t pulses = (int32_t)Y42_DegreesToPulses(degrees);

  if (pulses == 0)
  {
    return false;
  }

  /* 相对当前实时位置运动：不依赖零点参考，标定时最安全 */
  return SvcMotor_Y42SetPosPulses(addr, pulses, vel_rpm, acc,
                                  Y42_MOVE_REL_NOW);
}

bool SvcMotor_Y42GetPosPulses(uint8_t addr, int32_t *pulses)
{
  uint8_t     frame[Y42_FRAME_MAX];
  uint8_t     reply[SVC_RS485_REPLY_MAX];
  uint16_t    reply_len = 0U;
  uint8_t     len;
  Y42_Reply_t parsed;

  if (pulses == NULL)
  {
    return false;
  }

  len = Y42_BuildReadSysParam(frame, addr, Y42_INFO_CPOS);
  if (len == 0U)
  {
    return false;
  }

  if (!SvcRs485_Query(frame, len, addr, reply, sizeof(reply), &reply_len,
                      SVC_RS485_QUERY_TIMEOUT_MS))
  {
    return false;
  }

  /* 临时诊断：把 0x36 应答的原始字节打出来，确认字段布局。
     读数出现 8 位数（约 1280 圈）这种不可能的值，说明字节含义还没对。
     格式确认后删掉这一行。 */
  LogHexFrame(addr, reply, reply_len);

  if (!Y42_ParseReply(reply, reply_len, addr, &parsed))
  {
    /* 临时诊断：打印真实字节，确认应答格式。前 5 次打完就停，避免刷屏。 */
    static uint8_t s_fmt_err_logged = 0U;

    if (s_fmt_err_logged < 5U)
    {
      s_fmt_err_logged++;
      LogHexFrame(addr, reply, reply_len);
    }
    return false;
  }

  /* 0x36 的返回数据段是"符号 + 4 字节幅值"，负号在单独一个字节里，
     不是二补码 —— 直接按无符号数解析会永久丢失负方向。 */
  if (!Y42_DecodePosition(&parsed, pulses))
  {
    LogHexFrame(addr, reply, reply_len);
    return false;
  }

  return true;
}

bool SvcMotor_Y42GetPosDegrees(uint8_t addr, float *degrees)
{
  int32_t pulses;

  if (degrees == NULL)
  {
    return false;
  }
  if (!SvcMotor_Y42GetPosPulses(addr, &pulses))
  {
    return false;
  }

  *degrees = Y42_PositionToDegrees(pulses, SVC_MOTOR_Y42_FIRMWARE);
  return true;
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  在线检测：周期检查每台电机是否已超时无反馈
  */
static void SvcMotor_MonitorTask(void *argument)
{
  uint32_t now;
  uint8_t  i;

  (void)argument;

  for (;;)
  {
    osDelay(SVC_MOTOR_MONITOR_MS);

    now = HAL_GetTick();

    for (i = 1U; i <= SVC_MOTOR_DM_COUNT; i++)
    {
      DM3519_UpdateOnline(&s_dm[i], now, SVC_MOTOR_DM_OFFLINE_MS);

      if (s_prev_online[i] && (!s_dm[i].online))
      {
        s_prev_online[i] = false;
        log_printf("MOT", "DM%u offline", (unsigned)i);
      }
    }
  }
}
