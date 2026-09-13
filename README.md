# STM32G474VET6 电机控制固件项目

基于 STM32G474VET6 (Arm Cortex-M4F, 170MHz) 的数字电源控制器固件，支持 FOC 矢量控制（含 HFI 高频注入无传感器方案）、多外设协同（压缩机、风机、水泵）及 DGUS 串口屏人机交互。

## 版本说明

| 目录 | 说明 |
|---|---|
| `motor_control-v0.1/` | 基线版本 —— 初版 FOC + HFI 功能实现 |
| `motor_control-v1.0/` | 重构版本 —— 引入 MCAL 抽象层、FlashDB、模块解耦 |

## 硬件平台

- **MCU**: STM32G474VET6 (Cortex-M4F, 170MHz, 512KB Flash, 128KB SRAM)
- **开发板**: ATK-DMG474 + ATK-PD6010B 数字电源板
- **驱动方式**: 6 路 PWM 三相逆变器，三电阻/单电阻电流采样
- **传感器**: NTC 温度检测、模拟量传感器（压力/电流）、霍尔/无传感器

## 功能特性

- **FOC 矢量控制**：Id/Iq 双闭环 PI + Circle Limitation + SVPWM
- **无传感器观测器**：滑模观测器 (SMO) + 伦伯格观测器 (Luenberger)
- **HFI 高频注入**：脉振高频注入，支持零速/低速无传感器运行
- **CORDIC 加速**：硬件 CORDIC 协处理器进行 Park/Clarke 变换
- **RT-Thread RTOS**：多线程实时调度
- **Modbus 通信**：基于 UART 的 Modbus RTU 从站
- **DGUS 触摸屏**：串口屏 HMI，参数监控与设置
- **Monitor 监控**：运行时变量在线观测
- **告警系统**：过流、过温、欠压等多级保护

## 目录结构

```
├── motor_control-v0.1/        # 基线版本
│   ├── app/                   # 应用层（告警/压缩机/风机/水泵/HMI）
│   ├── bsp/                   # 板级支持包（启动/时钟/引脚/中断）
│   ├── config/                # 项目配置
│   ├── drivers/               # 驱动层（传感器/逆变器/电机/存储）
│   ├── hal/                   # HAL 抽象层（ADC/TIM/UART/GPIO/CORDIC）
│   ├── middleware/             # 中间件（Monitor/Modbus 协议）
│   ├── rtos/rt-thread/        # RT-Thread 内核
│   ├── utils/                 # 工具模块（FOC/PID/滤波器/CRC）
│   └── third_party/           # 第三方库（PY32T090 触摸库）
│
├── motor_control-v1.0/        # 重构版本
│   ├── app/                   # 应用层
│   ├── bsp/                   # 板级支持包
│   ├── drivers/               # 驱动层（motor/ain_sensor/key/inverter 等）
│   ├── hal/                   # HAL 抽象层
│   ├── mcal/                  # MCAL 层（CubeMX 生成的 STM32 HAL/LL 驱动）
│   ├── middleware/             # 中间件（Monitor/Modbus/OTA/协议）
│   ├── rtos/                  # RT-Thread + bare 裸机选项
│   ├── utils/                 # 工具模块（FOC/PID/CRC/滤波器）
│   ├── tests/                 # 单元测试
│   └── third_party/           # 第三方库（FlashDB/PID/Touch/PY32T090）
│
├── CHANGELOG.md               # 变更日志
└── README.md                  # 本文件
```

## 构建工具链

- **IDE**: Keil MDK-ARM (μVision 5)
- **工程文件**: `project/MDK-ARM/project.uvprojx`
- **HAL 库**: STM32Cube_FW_G4_V1.6.0
- **CubeMX**: `mcal/cubemx/stm32g474vet6/`（v1.0 版本）
- **编译器**: Arm Compiler 5/6 (AC5/AC6)

## 快速开始

1. 安装 Keil MDK-ARM 5 及 STM32G4 系列器件包
2. 打开对应版本的工程文件：`project/MDK-ARM/project.uvprojx`
3. 编译（Build）→ 下载（Download）到目标板
4. 通过 DGUS 串口屏或串口终端交互

## 架构分层

```
┌────────────────────────────────┐
│  app/         应用层（业务逻辑）  │
├────────────────────────────────┤
│  drivers/     驱动层（外设封装）  │
├────────────────────────────────┤
│  hal/         HAL 抽象层        │
├────────────────────────────────┤
│  mcal/        MCAL 层（v1.0）   │  ← STM32Cube HAL/LL 库
├────────────────────────────────┤
│  bsp/         板级支持包         │
└────────────────────────────────┘

依赖方向：app → drivers → hal → mcal → bsp（上层可调用下层，反之不可）
```

## 编码约定

- 层次依赖：`bsp ← hal ← drivers ← app`，上层可调用下层，反之不行
- 应用层通过 HAL 抽象层访问硬件，不直接调用 STM32 HAL/LL
- 换板子时只改 `bsp/` 和 `config/`，`hal/` 和 `drivers/` 不应改动
- 系统时钟 170MHz，所有定时参数基于此频率
- 固件版本号定义在 `config/project_config.h`

## 许可证

内部项目，保留所有权利。
