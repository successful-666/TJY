/**
  ******************************************************************************
  * @file    proto.c
  * @brief   协议帧的 CRC16、组帧与流式解析实现
  ******************************************************************************
  * @note    本模块是纯逻辑代码，不依赖 HAL / FreeRTOS。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "proto.h"

#include <string.h>

/* Private define ------------------------------------------------------------*/
/* 解析状态机的状态 */
#define ST_SYNC0                  0U
#define ST_SYNC1                  1U
#define ST_LEN_L                  2U
#define ST_LEN_H                  3U
#define ST_SEQ                    4U
#define ST_CMD                    5U
#define ST_PAYLOAD                6U
#define ST_CRC_L                  7U
#define ST_CRC_H                  8U

/* CRC16 初值与多项式（Modbus 风格，反射多项式 0xA001） */
#define CRC16_INIT                0xFFFFU
#define CRC16_POLY                0xA001U

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  在已有 CRC 基础上继续计算
  * @note   拆成"可累加"的形式，是为了能跨 hdr[] 和 payload[] 两块缓冲连续计算
  */
static uint16_t crc16_update(uint16_t crc, const uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint8_t  bit;

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i];

    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 1U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ CRC16_POLY);
      }
      else
      {
        crc = (uint16_t)(crc >> 1);
      }
    }
  }

  return crc;
}

/* Exported functions --------------------------------------------------------*/
uint16_t proto_crc16(const uint8_t *data, uint16_t len)
{
  return crc16_update(CRC16_INIT, data, len);
}

uint16_t proto_encode(uint8_t *out, uint16_t outSize, uint8_t seq, uint8_t cmd,
                      const uint8_t *payload, uint16_t len)
{
  uint16_t total;
  uint16_t crc;

  if ((out == NULL) || (len > PROTO_MAX_PAYLOAD))
  {
    return 0U;
  }

  total = (uint16_t)(PROTO_HEADER_LEN + len + PROTO_CRC_LEN);
  if (outSize < total)
  {
    return 0U;
  }

  out[0] = PROTO_SYNC0;
  out[1] = PROTO_SYNC1;
  out[2] = (uint8_t)(len & 0xFFU);
  out[3] = (uint8_t)((len >> 8) & 0xFFU);
  out[4] = seq;
  out[5] = cmd;

  if ((len > 0U) && (payload != NULL))
  {
    memcpy(&out[PROTO_HEADER_LEN], payload, (size_t)len);
  }

  /* CRC 覆盖 LEN / SEQ / CMD / PAYLOAD，不含 SYNC */
  crc = crc16_update(CRC16_INIT, &out[2], (uint16_t)(4U + len));

  out[PROTO_HEADER_LEN + len]        = (uint8_t)(crc & 0xFFU);
  out[PROTO_HEADER_LEN + len + 1U]   = (uint8_t)((crc >> 8) & 0xFFU);

  return total;
}

void proto_decoder_init(proto_decoder_t *dec)
{
  if (dec == NULL)
  {
    return;
  }

  memset(dec, 0, sizeof(*dec));
  dec->state = ST_SYNC0;
}

int proto_decoder_feed(proto_decoder_t *dec, uint8_t byte, proto_frame_t *frame)
{
  if (dec == NULL)
  {
    return -1;
  }

  switch (dec->state)
  {
  case ST_SYNC0:
    if (byte == PROTO_SYNC0)
    {
      dec->state = ST_SYNC1;
    }
    break;

  case ST_SYNC1:
    if (byte == PROTO_SYNC1)
    {
      dec->state = ST_LEN_L;
    }
    else if (byte == PROTO_SYNC0)
    {
      /* 连续收到 AA AA 的情况，保持在第二个字节上重新同步 */
      dec->state = ST_SYNC1;
    }
    else
    {
      dec->state = ST_SYNC0;
    }
    break;

  case ST_LEN_L:
    dec->hdr[0] = byte;
    dec->state  = ST_LEN_H;
    break;

  case ST_LEN_H:
    dec->hdr[1] = byte;
    dec->len    = (uint16_t)dec->hdr[0] | ((uint16_t)dec->hdr[1] << 8);

    if (dec->len > PROTO_MAX_PAYLOAD)
    {
      dec->state = ST_SYNC0;                 /* 长度非法，丢弃并重新找同步头 */
      return -1;
    }

    dec->state = ST_SEQ;
    break;

  case ST_SEQ:
    dec->hdr[2] = byte;
    dec->state  = ST_CMD;
    break;

  case ST_CMD:
    dec->hdr[3] = byte;
    dec->idx    = 0U;
    dec->state  = (dec->len > 0U) ? ST_PAYLOAD : ST_CRC_L;
    break;

  case ST_PAYLOAD:
    dec->payload[dec->idx] = byte;
    dec->idx++;

    if (dec->idx >= dec->len)
    {
      dec->state = ST_CRC_L;
    }
    break;

  case ST_CRC_L:
    dec->crcRecv = (uint16_t)byte;
    dec->state   = ST_CRC_H;
    break;

  case ST_CRC_H:
  {
    uint16_t crc;

    dec->crcRecv |= (uint16_t)((uint16_t)byte << 8);
    dec->state    = ST_SYNC0;                /* 无论成败都回到找同步头 */

    crc = crc16_update(CRC16_INIT, dec->hdr, 4U);
    crc = crc16_update(crc, dec->payload, dec->len);

    if (crc != dec->crcRecv)
    {
      return -1;
    }

    if (frame != NULL)
    {
      frame->seq = dec->hdr[2];
      frame->cmd = dec->hdr[3];
      frame->len = dec->len;

      if (dec->len > 0U)
      {
        memcpy(frame->payload, dec->payload, (size_t)dec->len);
      }
    }

    return 1;
  }

  default:
    dec->state = ST_SYNC0;
    break;
  }

  return 0;
}
