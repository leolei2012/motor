# Bootloader + IAP（OTA）

一句话：**常驻 bootloader**，支持两种 OTA 模式——**双槽 copy**（下到 APP2 → 校验 → copy APP1）
和 **单槽流式**（直接下到 APP1）；传输无关引擎 + Modbus FC41 适配器，CRC32 校验后跳转。

## 文档

| 编号 | 文件 | 内容 |
| --- | --- | --- |
| 01 | [architecture.md](01-architecture.md) | 目标 / 架构 / 决策(ADR) / 状态机 / 跳转 |
| 02 | [memory-layout.md](02-memory-layout.md) | Flash/RAM 分区 / 链接地址 / META 页结构 |
| 03 | [iap-protocol.md](03-iap-protocol.md) | MB-RTU FC41 协议 / bootloader 回调 / CRC32 |
| — | [规格符合性测试报告.md](规格符合性测试报告.md) | 规格基线 + 符合性测试用例 |

## 关联代码

- 固件工程：`bootloader-v1.0/`
- Modbus 协议栈：`bootloader-v1.0/third_party/MB-RTU/`（FC41 IAP 从机已集成）
- UART 收发：`bootloader-v1.0/third_party/uart_control/` + `hal_usart2`
- Bootloader 链接脚本：`bootloader-v1.0/bsp/linker/stm32g474xx_flash.sct`
