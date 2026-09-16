/**
  ******************************************************************************
  * @file    proto.h
  * @brief   上/下位机通信协议：帧格式定义、CRC16、组帧与流式解析
  ******************************************************************************
  * @note    本模块是纯逻辑代码，不依赖 HAL / FreeRTOS，可以单独拿到 PC 上测试。
  *
  * 帧格式（字节序：多字节字段为小端）：
  *
  *   +-------+-------+-------+-------+-------+-----------+-------+
  *   | SYNC0 | SYNC1 |  LEN  |  SEQ  |  CMD  |  PAYLOAD  | CRC16 |
  *   | 0xAA  | 0x55  | 2字节 | 1字节 | 1字节 |  N 字节   | 2字节 |
  *   +-------+-------+-------+-------+-------+-----------+-------+
  *
  *   LEN   ：PAYLOAD 的字节数（0 ~ PROTO_MAX_PAYLOAD），不含帧头也不含 CRC
  *   SEQ   ：序号，应答帧原样回填，用于上位机配对请求与应答
  *   CMD   ：命令字，应答帧的 CMD = 请求 CMD | PROTO_RSP_FLAG
  *   CRC16 ：Modbus 多项式 0xA001，初值 0xFFFF，覆盖 LEN/SEQ/CMD/PAYLOAD
  *           （即不含 SYNC0、SYNC1，也不含 CRC 自身）
  *
  ******************************************************************************
  */

#ifndef __PROTO_H__
#define __PROTO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define PROTO_SYNC0               0xAAU
#define PROTO_SYNC1               0x55U

#define PROTO_MAX_PAYLOAD         64U
#define PROTO_HEADER_LEN          6U   /* SYNC0 SYNC1 LEN_L LEN_H SEQ CMD */
#define PROTO_CRC_LEN             2U
#define PROTO_MAX_FRAME           (PROTO_HEADER_LEN + PROTO_MAX_PAYLOAD + PROTO_CRC_LEN)

/* 命令字（请求方向） */
#define PROTO_CMD_PING            0x01U   /* 请求：空 payload      应答：0x81 + 4 字节运行时间(ms, 小端) */
#define PROTO_CMD_ECHO            0x02U   /* 请求：任意 payload    应答：0x82 + 原样回显 */

/* 应答帧标志：应答 CMD = 请求 CMD | PROTO_RSP_FLAG */
#define PROTO_RSP_FLAG            0x80U

/* 错误应答：命令字无法识别时返回 */
#define PROTO_CMD_ERROR           0xFFU

/* Exported types ------------------------------------------------------------*/
/** 一个解析完成的帧 */
typedef struct
{
  uint8_t  seq;
  uint8_t  cmd;
  uint16_t len;
  uint8_t  payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

/** 流式解析器的状态，调用方按会话（连接）各持有自己的一个实例 */
typedef struct
{
  uint8_t  state;
  uint8_t  hdr[4];                        /* LEN_L LEN_H SEQ CMD，用于算 CRC */
  uint16_t len;
  uint16_t idx;
  uint8_t  payload[PROTO_MAX_PAYLOAD];
  uint16_t crcRecv;
} proto_decoder_t;

/* Exported functions prototypes ---------------------------------------------*/
uint16_t proto_crc16(const uint8_t *data, uint16_t len);

/**
  * @brief  把一帧数据编码进 out 缓冲区
  * @param  out     输出缓冲区
  * @param  outSize 输出缓冲区大小
  * @param  seq     序号
  * @param  cmd     命令字
  * @param  payload 负载，可为 NULL（当 len 为 0）
  * @param  len     负载长度
  * @retval 编码后的帧总长度；0 表示参数非法或缓冲区不够
  */
uint16_t proto_encode(uint8_t *out, uint16_t outSize, uint8_t seq, uint8_t cmd,
                      const uint8_t *payload, uint16_t len);

void proto_decoder_init(proto_decoder_t *dec);

/**
  * @brief  向解析器喂入一个字节
  * @param  dec   解析器实例
  * @param  byte  收到的字节
  * @param  frame 解析成功时填充，可为 NULL
  * @retval  1 = 收到一个完整且 CRC 正确的帧
  *          0 = 还需要更多字节
  *         -1 = 帧非法（长度越界或 CRC 错误），解析器已自动复位
  */
int proto_decoder_feed(proto_decoder_t *dec, uint8_t byte, proto_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __PROTO_H__ */
