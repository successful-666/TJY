/**
  ******************************************************************************
  * @file    y42.c
  * @brief   Y42 串口协议编解码实现
  ******************************************************************************
  * @note    命令字节序列对照 ZDT 官方例程逐条核对，并与手册 V1.1 的
  *          第 26 / 56 页格式说明比对过。
  *
  *          与官方例程的区别：官方例程在设备层直接调用 HAL_UART_Transmit_DMA，
  *          本移植只负责"打包成字节"，发送交给 BSP 层 —— 这样才能在发送前后
  *          正确切换 485 收发方向。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "y42.h"

#include <stddef.h>     /* NULL */

/* Private functions ---------------------------------------------------------*/
static bool AddrIsValid(uint8_t addr)
{
  /* 0 是广播地址，1~255 是实际电机地址。
     注意 addr 是 uint8_t，天然不可能超过 255，不需要再判上界。 */
  return ((addr == Y42_ADDR_BROADCAST) || (addr >= Y42_ADDR_MIN));
}

static void PutU16BE(uint8_t *out, uint16_t value)
{
  out[0] = (uint8_t)(value >> 8);
  out[1] = (uint8_t)(value & 0xFFU);
}

static void PutU32BE(uint8_t *out, uint32_t value)
{
  out[0] = (uint8_t)(value >> 24);
  out[1] = (uint8_t)(value >> 16);
  out[2] = (uint8_t)(value >> 8);
  out[3] = (uint8_t)(value & 0xFFU);
}

static uint32_t GetU32BE(const uint8_t *in)
{
  return ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) |
         ((uint32_t)in[2] << 8)  | (uint32_t)in[3];
}

/* Exported functions --------------------------------------------------------*/
uint8_t Y42_BuildEnControl(uint8_t *out, uint8_t addr, bool enable, bool sync)
{
  if ((out == NULL) || (!AddrIsValid(addr)))
  {
    return 0U;
  }

  out[0] = addr;
  out[1] = Y42_CODE_EN_CONTROL;
  out[2] = 0xABU;                       /* 辅助码 */
  out[3] = enable ? 1U : 0U;
  out[4] = sync ? 1U : 0U;
  out[5] = Y42_CHECK_BYTE;
  return 6U;
}

uint8_t Y42_BuildVelControl(uint8_t *out, uint8_t addr,
                            uint8_t dir, uint16_t vel_rpm,
                            uint8_t acc, bool sync)
{
  if ((out == NULL) || (!AddrIsValid(addr)) || (vel_rpm > Y42_VEL_MAX_RPM))
  {
    return 0U;
  }

  out[0] = addr;
  out[1] = Y42_CODE_VEL_CONTROL;
  out[2] = dir;
  PutU16BE(&out[3], vel_rpm);
  out[5] = acc;
  out[6] = sync ? 1U : 0U;
  out[7] = Y42_CHECK_BYTE;
  return 8U;
}

uint8_t Y42_BuildPosControl(uint8_t *out, uint8_t addr,
                            uint8_t dir, uint16_t vel_rpm, uint8_t acc,
                            uint32_t clk, uint8_t move_mode, bool sync)
{
  if ((out == NULL) || (!AddrIsValid(addr)) || (vel_rpm > Y42_VEL_MAX_RPM) ||
      (move_mode > Y42_MOVE_REL_NOW))
  {
    return 0U;
  }

  out[0]  = addr;
  out[1]  = Y42_CODE_POS_CONTROL;
  out[2]  = dir;
  PutU16BE(&out[3], vel_rpm);
  out[5]  = acc;
  PutU32BE(&out[6], clk);
  out[10] = move_mode;
  out[11] = sync ? 1U : 0U;
  out[12] = Y42_CHECK_BYTE;
  return 13U;
}

uint8_t Y42_BuildStopNow(uint8_t *out, uint8_t addr, bool sync)
{
  if ((out == NULL) || (!AddrIsValid(addr)))
  {
    return 0U;
  }

  out[0] = addr;
  out[1] = Y42_CODE_STOP_NOW;
  out[2] = 0x98U;                       /* 辅助码 */
  out[3] = sync ? 1U : 0U;
  out[4] = Y42_CHECK_BYTE;
  return 5U;
}

uint8_t Y42_BuildSyncMotion(uint8_t *out, uint8_t addr)
{
  if ((out == NULL) || (!AddrIsValid(addr)))
  {
    return 0U;
  }

  out[0] = addr;
  out[1] = Y42_CODE_SYNC_MOTION;
  out[2] = 0x66U;                       /* 辅助码 */
  out[3] = Y42_CHECK_BYTE;
  return 4U;
}

uint8_t Y42_BuildResetCurPos(uint8_t *out, uint8_t addr)
{
  if ((out == NULL) || (!AddrIsValid(addr)))
  {
    return 0U;
  }

  out[0] = addr;
  out[1] = Y42_CODE_RESET_CURPOS;
  out[2] = 0x6DU;                       /* 辅助码 */
  out[3] = Y42_CHECK_BYTE;
  return 4U;
}

uint8_t Y42_BuildReadSysParam(uint8_t *out, uint8_t addr, uint8_t info_code)
{
  if ((out == NULL) || (!AddrIsValid(addr)))
  {
    return 0U;
  }

  out[0] = addr;
  out[1] = info_code;
  out[2] = Y42_CHECK_BYTE;
  return 3U;
}

uint8_t Y42_BuildReadState(uint8_t *out, uint8_t addr)
{
  if ((out == NULL) || (!AddrIsValid(addr)))
  {
    return 0U;
  }

  out[0] = addr;
  out[1] = Y42_CODE_READ_STATE;
  out[2] = 0x7AU;                       /* 辅助码 */
  out[3] = Y42_CHECK_BYTE;
  return 4U;
}

uint8_t Y42_BuildReadConf(uint8_t *out, uint8_t addr)
{
  if ((out == NULL) || (!AddrIsValid(addr)))
  {
    return 0U;
  }

  out[0] = addr;
  out[1] = Y42_CODE_READ_CONF;
  out[2] = 0x6CU;                       /* 辅助码 */
  out[3] = Y42_CHECK_BYTE;
  return 4U;
}

bool Y42_ParseAck(const uint8_t *buf, uint16_t len,
                  uint8_t expect_addr, uint8_t expect_code)
{
  if ((buf == NULL) || (len != 4U))
  {
    return false;
  }

  /* 控制类应答固定 4 字节：地址 + 功能码 + 状态 + 校验 */
  return ((buf[0] == expect_addr) &&
          (buf[1] == expect_code) &&
          (buf[2] == Y42_ACK_STATUS_OK) &&
          (buf[3] == Y42_CHECK_BYTE));
}

bool Y42_ParseReply(const uint8_t *buf, uint16_t len,
                    uint8_t expect_addr, Y42_Reply_t *reply)
{
  uint16_t i;
  uint16_t data_len;

  if ((buf == NULL) || (reply == NULL))
  {
    return false;
  }
  if ((len < Y42_REPLY_MIN_LEN) ||
      (len > (uint16_t)(Y42_REPLY_DATA_MAX + 3U)))
  {
    return false;
  }
  /* 地址匹配 + 末字节是校验码：判断"这帧是不是给我的"的两个依据 */
  if ((buf[0] != expect_addr) || (buf[len - 1U] != Y42_CHECK_BYTE))
  {
    return false;
  }

  /* 返回数据段 = 功能码与校验码之间的所有字节，长度随命令而异 */
  data_len = (uint16_t)(len - 3U);

  reply->addr     = buf[0];
  reply->code     = buf[1];
  reply->data_len = (uint8_t)data_len;

  for (i = 0U; i < data_len; i++)
  {
    reply->data[i] = buf[2U + i];
  }

  return true;
}

bool Y42_DecodePosition(const Y42_Reply_t *reply, int32_t *position)
{
  uint32_t magnitude;

  if ((reply == NULL) || (position == NULL))
  {
    return false;
  }
  /* 0x36 的返回数据段固定 5 字节：符号(1) + 位置(4, 大端) */
  if (reply->data_len != Y42_POS_REPLY_DATA_LEN)
  {
    return false;
  }

  magnitude = GetU32BE(&reply->data[1]);

  /* 符号是独立的一个字节，不是二补码 —— 手册 5.5.13 节明确写了 00/01 */
  if (reply->data[0] == Y42_SIGN_NEGATIVE)
  {
    *position = -(int32_t)magnitude;
  }
  else
  {
    *position = (int32_t)magnitude;
  }

  return true;
}

float Y42_PositionToDegrees(int32_t position, uint8_t firmware)
{
  if (firmware == Y42_FW_X)
  {
    /* X 固件：角度 = 位置 / 10 */
    return (float)position / 10.0f;
  }

  /* Emm 固件：0~65535 对应一圈 0~360° */
  return ((float)position * 360.0f) / 65536.0f;
}

uint32_t Y42_DegreesToPulses(float degrees)
{
  /* 位置模式命令用的是"输入脉冲数"，3200 脉冲 = 一圈（16 细分）。
     注意这与读取回来的编码器计数不是同一个量纲。 */
  return (uint32_t)((degrees * (float)Y42_CMD_PULSE_PER_REV) / 360.0f);
}
