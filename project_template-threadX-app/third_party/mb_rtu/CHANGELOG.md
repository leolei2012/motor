# Changelog

本文件记录 Modbus RTU 协议栈的所有重要变更。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [Semantic Versioning](https://semver.org/lang/zh-CN/)。

---

## [1.0.0] - 2026-08-22

### 新增

- Master / Slave 角色解耦，拆分为独立的头文件与源文件（`modbus_slave.*`、`modbus_master.*`）。
- 公共层 `modbus_common.*`：功能码 / 异常码 / 错误码枚举 / `mb_base` / 发送回调 / CRC16。
- 从机数据绑定支持回调方式（`on_read` / `on_write`），回调优先于连续数组，适配非连续内存、外设映射、实时计算量等场景。
- IAP 升级（自定义功能码 `0x41`）：单功能码 + 子命令（START / DATA / END / STATUS），从机按块号顺序「接数据」并回调，主机提供配套请求函数。

### 变更

- 发送回调增加 `void *ctx`：`mb_send_func_t` 改为 `(buf, len, ctx)`，`mb_base` 增加 `ctx` 字段，`mb_base_init` / `mb_master_init` 增加 `ctx` 参数（回调里通过 `ctx` 找回实例，支持多实例）。
- 对象指针参数统一命名 `self`：原 `hdl` 全部改为 `self`（纯命名重构，不影响 ABI）。
- IAP `START` 子命令新增 `fw_crc32(4B)` 字段，携带固件镜像 CRC32（标准 CRC-32，`0xEDB88320`）；`mb_master_iap_start` 增加 `fw_crc32` 参数，`mb_iap_start_cb_t` 回调签名改为 `(uint32_t total_size, uint32_t crc32)`。CRC32 校验由用户在 `on_end` 回调中自行比对，库保持「只接数据」。
- 数据模型：`uint16_t **p_data` → `uint16_t *data`、`uint8_t **p_data` → `uint8_t *data`，去除冗余指针数组。
- 删除全局写通知回调 `on_write_coil` / `on_write_reg`，业务逻辑并入段级 `on_write`。
- 删除单角色抽象宏（`mb_handle` / `mb_init` / `mb_rx_frame` / `mb_tick`），调用方按角色直接使用 `mb_slave_*` / `mb_master_*`。
- CRC 内部函数加 `mb_` 前缀（`mb_crc16` / `mb_append_crc` / `mb_check_crc`），实现移入 `modbus_common.c`。
- CRC 函数声明并入 `modbus_common.h`（公开），移除私有头 `src/modbus_internal.h`。
- 注释统一为 Doxygen 风格（`@file` / `@brief` / `@param` / `@return` / `/**< */` / `@name` 分组），覆盖全部头文件与源文件，便于生成 API 文档。

### 移除

- 废弃 `include/modbus/modbus.h` 伞形头与 `src/modbus.c` 单文件实现，改为按角色分文件组织。
