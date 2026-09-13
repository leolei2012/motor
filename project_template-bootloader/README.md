# STM32G474VET6 开发板模板工程

基于 **STM32G474VET6**（Arm Cortex-M4F @ 170 MHz）的裸机固件模板工程，Keil MDK-ARM 构建，按分层架构组织，不绑定具体 RTOS，可作为新产品 / 新板卡的起点骨架。

## 特性

- **分层架构**：`bsp → hal → drivers → app`，依赖单向无环，换板只改 `bsp/`
- **裸机运行**：`main()` 手动初始化各层 + 主循环周期轮询，无 RTOS 依赖
- **可复用模块集**：
  - 驱动：模拟量传感器（ADC + EMA 滤波）
  - 应用示例：`alarm_system` 告警系统
  - 工具：alarm / check_algorithm / debug / ntc_temp / platform / rcm / ring_buffer
  - 第三方：filter、pid
- **规范落地**：目录形态 C、分布式组合根、构造注入，详见 `docs/规范/`

## 硬件平台

| 项 | 说明 |
|---|---|
| MCU | STM32G474VET6（Cortex-M4F，170 MHz） |
| 原理图 / 手册 | `docs/原理图/`（DMG474、ATK-PD6010B） |
| 数据手册 | `docs/datasheet/` |

## 目录结构

```text
app/         应用层（业务逻辑）
drivers/     驱动层（外设封装）
hal/         HAL 抽象层（外设实例薄封装）
bsp/         板级支持包（入口 / 时钟 / 引脚 / NVIC / 启动 / 链接脚本）
config/      项目配置
utils/       工具模块
tests/       单元测试 & 集成测试
third_party/ 第三方库
rtos/        RTOS 占位（默认裸机，不指定）
docs/        文档与规范
```

## 构建

### 前置条件

1. **Keil MDK-ARM**（μVision 5）
2. **mcal（厂商 SDK）**：`mcal/` 被 `.gitignore` 忽略，需手动放置：
   - STM32Cube_FW_G4_V1.6.0 → `mcal/STM32Cube_FW_G4_V1.6.0/`
   - CubeMX 配置 → `mcal/cubemx/`

### 编译

用 Keil 打开 `project/MDK-ARM/project.uvprojx` 后编译即可。

## 快速上手

固件入口在 `bsp/src/main.c`。裸机启动顺序：

```text
bsp_board_early_init()   时钟 / HAL 初始化
hal_init()               外设初始化（GPIO / ADC / TIM6）
drv_init()               驱动实例化 + 注入
app_init()               应用实例化 + 注入

主循环（while (1)）
  ├── LED 心跳（500ms 翻转）
  ├── drv_ain_sensor_poll()      模拟量物理量转换（100ms）
  └── app_alarm_system_poll()    告警检测（1000ms）
```

模拟量采样由 TIM6 1kHz 中断驱动（`bsp/src/bsp_isr.c` 的 `TIM6_DAC_IRQHandler`），
主循环只做物理量转换，不阻塞。

## 编码约定

- 层次依赖单向：上层可调用下层，反之不行；应用层不直接碰 STM32 HAL/LL
- 对象用带 tag 的 `struct`，方法为「自由函数 + `self`」，依赖走 `init` 构造注入
- 详见 `docs/规范/architecture.md` 与 `docs/规范/coding_standard.md`

## 作者

见 [AUTHORS](./AUTHORS)。

## 变更记录

见 [CHANGELOG.md](./CHANGELOG.md)。
