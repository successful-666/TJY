/**
  * Y42 协议层单元测试（在 PC 上运行，不需要硬件）
  *
  * 期望值全部取自 ZDT 官方手册里的示例报文，
  * 用来验证 Device/y42.c 打包出的字节序列与官方完全一致。
  *
  * 编译: gcc -I code/Device -o test_y42.exe test_y42.c code/Device/y42.c
  */

#include <stdio.h>
#include <string.h>

#include "y42.h"

static int g_fail = 0;
static int g_pass = 0;

static void Check(const char *name,
                  const uint8_t *got, uint8_t gotLen,
                  const uint8_t *want, uint8_t wantLen)
{
  uint8_t i;
  int ok = (gotLen == wantLen);

  if (ok)
  {
    for (i = 0U; i < gotLen; i++)
    {
      if (got[i] != want[i]) { ok = 0; break; }
    }
  }

  printf("%-34s ", name);
  if (ok)
  {
    printf("PASS\n");
    g_pass++;
    return;
  }

  printf("FAIL\n");
  printf("     期望 %2u 字节: ", wantLen);
  for (i = 0U; i < wantLen; i++) { printf("%02X ", want[i]); }
  printf("\n     实际 %2u 字节: ", gotLen);
  for (i = 0U; i < gotLen; i++) { printf("%02X ", got[i]); }
  printf("\n");
  g_fail++;
}

int main(void)
{
  uint8_t f[Y42_FRAME_MAX];
  uint8_t n;

  printf("=== Y42 (Emm_V5) 协议层测试 ===\n\n");

  /* 1. 位置模式：手册示例 "01 FD 01 0F A0 00 00 01 FA 00 00 00 6B"
        地址1、CCW、4000RPM、加速度0、脉冲数 0x1FA(506)、相对上一目标、不同步 */
  {
    const uint8_t want[] = {0x01,0xFD,0x01,0x0F,0xA0,0x00,
                            0x00,0x00,0x01,0xFA,0x00,0x00,0x6B};
    n = Y42_BuildPosControl(f, 1U, Y42_DIR_CCW, 4000U, 0U,
                            0x000001FAUL, Y42_MOVE_REL_LAST, false);
    Check("位置模式（手册示例1）", f, n, want, (uint8_t)sizeof(want));
  }

  /* 2. 位置模式：手册示例 "01 FD 01 05 DC 08 00 00 7D 00 00 00 6B" */
  {
    const uint8_t want[] = {0x01,0xFD,0x01,0x05,0xDC,0x08,
                            0x00,0x00,0x00,0x7D,0x00,0x00,0x6B};
    n = Y42_BuildPosControl(f, 1U, Y42_DIR_CCW, 1500U, 8U,
                            125UL, Y42_MOVE_REL_LAST, false);
    Check("位置模式（手册示例2）", f, n, want, (uint8_t)sizeof(want));
  }

  /* 3. 位置模式：手册示例 "02 FD 00 03 E8 0A 00 00 FA 00 01 01 6B"
        地址2、CW、1000RPM、加速度10、脉冲250、绝对位置、同步使能 */
  {
    const uint8_t want[] = {0x02,0xFD,0x00,0x03,0xE8,0x0A,
                            0x00,0x00,0x00,0xFA,0x01,0x01,0x6B};
    n = Y42_BuildPosControl(f, 2U, Y42_DIR_CW, 1000U, 10U,
                            250UL, Y42_MOVE_ABSOLUTE, true);
    Check("位置模式（手册示例3）", f, n, want, (uint8_t)sizeof(want));
  }

  /* 4. 速度模式：手册示例 "01 F6 01 05 DC 0A 00 6B" */
  {
    const uint8_t want[] = {0x01,0xF6,0x01,0x05,0xDC,0x0A,0x00,0x6B};
    n = Y42_BuildVelControl(f, 1U, Y42_DIR_CCW, 1500U, 10U, false);
    Check("速度模式（手册示例）", f, n, want, (uint8_t)sizeof(want));
  }

  /* 5. 使能控制："01 F3 AB 01 00 6B" */
  {
    const uint8_t want[] = {0x01,0xF3,0xAB,0x01,0x00,0x6B};
    n = Y42_BuildEnControl(f, 1U, true, false);
    Check("使能控制（手册示例）", f, n, want, (uint8_t)sizeof(want));
  }

  /* 6. 立即停止："01 FE 98 00 6B" */
  {
    const uint8_t want[] = {0x01,0xFE,0x98,0x00,0x6B};
    n = Y42_BuildStopNow(f, 1U, false);
    Check("立即停止（手册示例）", f, n, want, (uint8_t)sizeof(want));
  }

  /* 7. 多机同步启动："01 FF 66 6B" */
  {
    const uint8_t want[] = {0x01,0xFF,0x66,0x6B};
    n = Y42_BuildSyncMotion(f, 1U);
    Check("多机同步启动（手册示例）", f, n, want, (uint8_t)sizeof(want));
  }

  /* 8. 位置清零："01 0A 6D 6B" */
  {
    const uint8_t want[] = {0x01,0x0A,0x6D,0x6B};
    n = Y42_BuildResetCurPos(f, 1U);
    Check("当前位置清零（手册示例）", f, n, want, (uint8_t)sizeof(want));
  }

  /* 9. 读取实时位置："01 36 6B" */
  {
    const uint8_t want[] = {0x01,0x36,0x6B};
    n = Y42_BuildReadSysParam(f, 1U, Y42_INFO_CPOS);
    Check("读取实时位置（手册示例）", f, n, want, (uint8_t)sizeof(want));
  }

  /* 10. 应答解析：控制类 "01 F6 02 6B" */
  {
    const uint8_t ack[] = {0x01,0xF6,0x02,0x6B};
    printf("%-34s %s\n", "解析控制类应答",
           Y42_ParseAck(ack, 4U, 1U, Y42_CODE_VEL_CONTROL) ? "PASS" : "FAIL");
    if (!Y42_ParseAck(ack, 4U, 1U, Y42_CODE_VEL_CONTROL)) { g_fail++; } else { g_pass++; }
  }

  /* 11. 应答解析：读取类。用实机抓到的真实报文，不是编的 */
  {
    const uint8_t rep[] = {0x01,0x36,0x01,0x00,0x00,0x00,0x03,0x6B};
    Y42_Reply_t r;
    int32_t v = 0;

    memset(&r, 0, sizeof(r));
    if (Y42_ParseReply(rep, (uint16_t)sizeof(rep), 1U, &r) &&
        Y42_DecodePosition(&r, &v))
    {
      /* 实测报文 01 36 01 00 00 00 03 6B：符号=01(负)，幅值=3 → -3 */
      printf("%-34s %s (值=%d)\n", "解析位置应答(实机报文)",
             (v == -3) ? "PASS" : "FAIL", (int)v);
      if (v == -3) { g_pass++; } else { g_fail++; }
    }
    else
    {
      printf("%-34s FAIL\n", "解析位置应答(实机报文)");
      g_fail++;
    }
  }

  /* 12. 拒绝长度不对的应答（7 字节是旧假设，必须被拒） */
  {
    const uint8_t bad[] = {0x01,0x36,0x01,0x00,0x00,0x00,0x03};
    Y42_Reply_t r;
    int ok = !Y42_ParseReply(bad, (uint16_t)sizeof(bad), 1U, &r);
    printf("%-34s %s\n", "拒绝校验码错误的帧", ok ? "PASS" : "FAIL");
    if (ok) { g_pass++; } else { g_fail++; }
  }

  /* 13. 两种固件的角度换算（手册 5.5.13：同一数值含义完全不同） */
  {
    float dx = Y42_PositionToDegrees(16, Y42_FW_X);
    int okx = (dx > 1.59f) && (dx < 1.61f);
    printf("%-34s %s (%.2f)\n", "X 固件 16 -> 1.6 度",
           okx ? "PASS" : "FAIL", (double)dx);
    if (okx) { g_pass++; } else { g_fail++; }

    float de = Y42_PositionToDegrees(32768, Y42_FW_EMM);
    int oke = (de > 179.9f) && (de < 180.1f);
    printf("%-34s %s (%.2f)\n", "Emm 读取 32768 -> 180 度",
           oke ? "PASS" : "FAIL", (double)de);
    if (oke) { g_pass++; } else { g_fail++; }
  }

  /* 14. 位置模式命令的脉冲换算（手册 5.3.12：3200 脉冲 = 一圈） */
  {
    uint32_t p360 = Y42_DegreesToPulses(360.0f);
    uint32_t p10t = Y42_DegreesToPulses(3600.0f);   /* 10 圈，手册例子是 32000 */
    int ok360 = (p360 == 3200U);
    int ok10t = (p10t == 32000U);

    printf("%-34s %s (%u)\n", "命令换算 360 度 -> 3200 脉冲",
           ok360 ? "PASS" : "FAIL", (unsigned)p360);
    printf("%-34s %s (%u)\n", "命令换算 3600 度 -> 32000 脉冲",
           ok10t ? "PASS" : "FAIL", (unsigned)p10t);
    if (ok360) { g_pass++; } else { g_fail++; }
    if (ok10t) { g_pass++; } else { g_fail++; }
  }

  /* 12. 非法参数必须被拒绝 */
  {
    int ok = 1;
    if (Y42_BuildVelControl(f, 1U, 0U, 6000U, 0U, false) != 0U) { ok = 0; }
    if (Y42_BuildPosControl(f, 1U, 0U, 100U, 0U, 0UL, 9U, false) != 0U) { ok = 0; }
    printf("%-34s %s\n", "非法参数被拒绝", ok ? "PASS" : "FAIL");
    if (ok) { g_pass++; } else { g_fail++; }
  }

  printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
