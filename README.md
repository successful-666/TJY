# Car_Robot 主控固件

基于 STM32F407VET6 的移动底盘主控板固件：以太网 TCP 作为上位机接口，
CAN 与 RS485 分别驱动机器人的行走电机与转向电机。

## 硬件

| 部件 | 接口 | 说明 |
|---|---|---|
| 主控 | — | STM32F407VET6，168MHz，LQFP100 |
| 行走电机 × 4 | CAN1 (1 Mbps) | 达妙 DM3519 |
| 转向电机 × 2 | RS485 / USART2 | 张大头 Y42 闭环步进 |
| 上位机 | RMII 以太网 | 静态 IP `192.168.1.2`，TCP 端口 5000 |
| 调试口 | USART1 | 115200-8-N-1，带命令行 |

## 软件架构

依赖严格单向下行，每层只依赖下面一层：

```
App  →  Service  →  Device  →  BSP  →  HAL
```

| 层 | 目录 | 职责 |
|---|---|---|
| App | `code/App/` | 任务编排、TCP 服务、串口命令行、底盘运动学 |
| Service | `code/Service/` | CAN / RS485 收发服务、电机业务与状态缓存 |
| Device | `code/Device/` | 设备协议编解码（纯逻辑，零硬件依赖） |
| BSP | `code/BSP/` | CAN / UART 外设封装，唯一接触 HAL 的地方 |
| Core | `code/Core/` | STM32CubeMX 生成代码 |

`Device` 层不包含任何 HAL 或 FreeRTOS 头文件，因此可以拿到 PC 上做单元测试
（见 `AI工作层/test_y42.c`、`AI工作层/test_chassis.c`）。

## 调试命令行

通过调试串口（USART1，115200）直接控制电机。**小写 = 485/Y42，大写 = CAN/DM3519**。

| 命令 | 说明 |
|---|---|
| `?` | 帮助 |
| `r <id>` / `e <id>` / `d <id>` / `s <id>` / `z <id>` | Y42：读位置 / 使能 / 失能 / 停止 / 位置清零 |
| `v <id> <rpm>` | Y42 速度模式（正负表示方向） |
| `p <id> <pulses>` / `a <id> <pulses>` | Y42 位置模式：相对当前位置 / 绝对位置 |
| `q <id> <deg>` | Y42 相对走一个角度 |
| `t <offset_deg>` | 转向偏移，0=回正 |
| `E <id>` / `D <id>` / `S <id>` | DM3519：使能 / 失能 / 速度归零 |
| `V <id> <rpm>` | DM3519 速度模式 |
| `R <id>` | DM3519 状态 |

测试脚本见 `AI工作层/com_test.ps1`。

## 编译

用 Keil MDK-ARM 打开 `code/MDK-ARM/Car_Robot.uvprojx`。

> **注意**：工程必须开启 **Use MicroLIB**（Options for Target → Target）。
> 关闭时 ARM 标准 C 库会在 `main()` 之前走半主机（semihosting），
> 导致脱机运行时直接 HardFault。

## 进度

- [x] 以太网 + LwIP 链路
- [x] FreeRTOS 任务与日志框架
- [x] RS485 / Y42：位置读取、运动控制（实机验证通过）
- [x] CAN / DM3519：驱动与服务层（待实机验证）
- [x] 底盘差速运动学（含单元测试）
- [ ] 转向基准角实车标定
- [ ] 上位机 TCP 协议定版

详细设计说明见 `AI工作层/架构设计.md`。
