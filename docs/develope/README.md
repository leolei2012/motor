# 开发文档工作区

> 本目录是 bootloader / OTA 等新特性的**设计文档工作区**，与 `motor_control-v1.0/docs/`（项目已有资料）分开。

## 目录

| 目录 | 内容 |
| --- | --- |
| [`bootloader/`](bootloader/README.md) | Bootloader + IAP（OTA）设计 |

## 约定

- 语言：中文；代码 / 寄存器名 / 命令用英文。
- 文档按主题分目录，主题内按 01 / 02 / 03 编号，覆盖「架构」「内存布局」「协议」三类。
- **以代码和头文件为准**：文档用于对齐意图；实现后如有偏差，以代码为准并回写文档。
- 关键决策用 ADR 风格记录（决策 + 理由 + 被否方案）。

## 状态

| 文档 | 状态 |
| --- | --- |
| bootloader/01-architecture | 已评审，待实现 |
| bootloader/02-memory-layout | 已评审，待实现 |
| bootloader/03-iap-protocol | 已评审，待实现 |
