/**
  * 底盘运动学单元测试（在 PC 上运行，不需要硬件）
  *
  * 期望值来自参考工程 Frame_work/APP/app.c 的控制逻辑。
  * 注意：wheel_rpm 是"电机坐标系"的值，已乘过方向系数，
  *       所以左右镜像安装时物理同向的两个轮子符号相反。
  *
  * 编译: gcc -I code/App -o test_chassis.exe test_chassis.c code/App/chassis.c
  */

#include <stdio.h>
#include <string.h>

#include "chassis.h"

static int g_fail = 0;
static int g_pass = 0;

static void Check(const char *name, int got, int want)
{
  printf("%-40s %s", name, (got == want) ? "PASS\n" : "FAIL\n");
  if (got == want) { g_pass++; }
  else { printf("     期望 %d, 实际 %d\n", want, got); g_fail++; }
}

static void CheckWheels(const char *name, const Chassis_Output_t *o,
                        int fl, int fr, int rl, int rr)
{
  int ok = (o->wheel_rpm[CHASSIS_WHEEL_FL] == fl) &&
           (o->wheel_rpm[CHASSIS_WHEEL_FR] == fr) &&
           (o->wheel_rpm[CHASSIS_WHEEL_RL] == rl) &&
           (o->wheel_rpm[CHASSIS_WHEEL_RR] == rr);

  printf("%-40s %s\n", name, ok ? "PASS" : "FAIL");
  if (ok) { g_pass++; }
  else
  {
    printf("     期望 FL=%d FR=%d RL=%d RR=%d\n", fl, fr, rl, rr);
    printf("     实际 FL=%d FR=%d RL=%d RR=%d\n",
           o->wheel_rpm[CHASSIS_WHEEL_FL], o->wheel_rpm[CHASSIS_WHEEL_FR],
           o->wheel_rpm[CHASSIS_WHEEL_RL], o->wheel_rpm[CHASSIS_WHEEL_RR]);
    g_fail++;
  }
}

int main(void)
{
  Chassis_Output_t o;

  printf("=== 底盘运动学测试 ===\n\n");

  /* 用默认配置（沿用参考工程的轮子方向系数与限幅值） */
  Chassis_Init(NULL);

  /* 1. 直行：两侧同速，方向系数使电信号相反 */
  memset(&o, 0, sizeof(o));
  Chassis_Mix(10, 0, 0, &o);
  CheckWheels("直行 10rpm", &o, 10, -10, 10, -10);

  /* 2. 原地左转：linear=0，左轮后退右轮前进。
        右轮 dir=-1，所以电机坐标系里四个轮子都是 -10，
        物理上则是"左 -10、右 +10"。 */
  memset(&o, 0, sizeof(o));
  Chassis_Mix(0, 10, 0, &o);
  CheckWheels("原地左转 10rpm", &o, -10, -10, -10, -10);

  /* 3. 边前进边左转：linear=10, yaw=5 */
  memset(&o, 0, sizeof(o));
  Chassis_Mix(10, 5, 0, &o);
  CheckWheels("前进10+左转5", &o, 5, -15, 5, -15);

  /* 4. 限幅：超过 20rpm 被饱和到 ±20 */
  memset(&o, 0, sizeof(o));
  Chassis_Mix(100, 0, 0, &o);
  CheckWheels("限幅 100rpm -> 20rpm", &o, 20, -20, 20, -20);

  /* 5. 转向：默认基准角 0，偏移 30 度，右转向机构镜像 */
  memset(&o, 0, sizeof(o));
  Chassis_Mix(0, 0, 30, &o);
  Check("转向偏移 +30 度 -> 左机构", o.steer_angle_deg[CHASSIS_STEER_LEFT], 30);
  Check("转向偏移 +30 度 -> 右机构", o.steer_angle_deg[CHASSIS_STEER_RIGHT], -30);

  /* 6. 转向偏移限幅：±45 度 */
  memset(&o, 0, sizeof(o));
  Chassis_Mix(0, 0, 100, &o);
  Check("转向偏移限幅 -> 左机构", o.steer_angle_deg[CHASSIS_STEER_LEFT], 45);
  Check("转向偏移限幅 -> 右机构", o.steer_angle_deg[CHASSIS_STEER_RIGHT], -45);

  /* 7. 用参考工程的实测基准角（212 / 102）覆盖默认值 */
  {
    Chassis_Config_t cfg;
    const Chassis_Config_t *cur = Chassis_GetConfig();

    cfg = *cur;
    cfg.steer_ref_deg[CHASSIS_STEER_LEFT]  = 212;
    cfg.steer_ref_deg[CHASSIS_STEER_RIGHT] = 102;
    Chassis_Init(&cfg);

    memset(&o, 0, sizeof(o));
    Chassis_Mix(0, 0, 30, &o);
    Check("基准角212/102 + 偏移30 -> 左", o.steer_angle_deg[CHASSIS_STEER_LEFT], 242);
    Check("基准角212/102 + 偏移30 -> 右", o.steer_angle_deg[CHASSIS_STEER_RIGHT], 72);

    /* 8. 回正：偏移 0 时两机构都回到各自基准角 */
    memset(&o, 0, sizeof(o));
    Chassis_Mix(0, 0, 0, &o);
    Check("回正 -> 左机构", o.steer_angle_deg[CHASSIS_STEER_LEFT], 212);
    Check("回正 -> 右机构", o.steer_angle_deg[CHASSIS_STEER_RIGHT], 102);
  }

  printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
