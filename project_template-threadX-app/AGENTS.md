# STM32G474VET6 开发板模板工程

基于 STM32G474VET6 的裸机开发板模板工程。保留分层骨架（bsp / hal / drivers / app）与一组可复用模块作为起点，不绑定具体 RTOS，业务逻辑按需裁剪。

## 硬件平台
- **MCU**: STM32G474VET6 (Arm Cortex-M4F, 170 MHz)
- **原理图 / 手册**: `docs/原理图/`（DMG474 原理图与硬件参考手册、ATK-PD6010B 电源板手册）
- **芯片数据手册**: `docs/datasheet/`

## 构建工具链
- **IDE**: Keil MDK-ARM (μVision)
- **工程文件**: `project/MDK-ARM/project.uvprojx`
- **MCAL**: STM32Cube_FW_G4_V1.6.0

> **注意**：`mcal/`（厂商 SDK）体积大、被 `.gitignore` 忽略，不在仓库内。
> 构建前需将 STM32Cube_FW_G4_V1.6.0 放到 `mcal/STM32Cube_FW_G4_V1.6.0/`，
> CubeMX 配置放到 `mcal/cubemx/`。

## RTOS
- 模板默认**裸机**运行；`rtos/` 为占位目录，需要时可放入 RT-Thread / FreeRTOS 等内核，不绑定具体 RTOS。

## 项目架构

```
app/         应用层：业务逻辑，不直接操作硬件
  └── alarm_system/   告警系统（示例模块，基于 utils/alarm）
drivers/     驱动层：封装外设操作
  └── ain_sensor/     模拟量传感器（ADC + EMA 滤波）
hal/         HAL 抽象层：外设实例的薄封装（adc1、gpio、tim6），不含板级细节
bsp/         板级支持包：入口 (main.c)、时钟树、引脚映射、NVIC、启动文件、链接脚本（与 PCB 绑定）
config/      项目配置（project_config.h 含 Keil Configuration Wizard 注解）
utils/       工具模块（alarm、check_algorithm、debug、ntc_temp、platform、rcm、ring_buffer）
tests/       单元测试 & 集成测试
third_party/ 第三方库（filter、pid）
rtos/        RTOS 占位（默认裸机，不指定具体内核）
```

> 分层与依赖规则详见 `docs/规范/architecture.md`、编码约定见 `docs/规范/coding_standard.md`。

## 编码约定
- **层次依赖**: bsp ← hal ← drivers ← app。上层可以调用下层，反之不行
- 应用层通过 HAL 抽象层访问硬件，不直接调用 STM32 HAL 或 LL
- 换板子时只改 `bsp/`（必要时 `hal/`），`drivers/`、`app/` 不应改动
- 裸机：`main()` 按 `bsp_board_early_init → hal_init → drv_init → app_init` 顺序初始化，主循环周期轮询
- 系统时钟 170 MHz，所有定时参数基于此频率
- 固件版本号定义在 `config/project_config.h`
