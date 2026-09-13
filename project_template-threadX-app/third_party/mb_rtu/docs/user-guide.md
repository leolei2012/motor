# Modbus RTU 协议栈 · 用户指南

> 轻量级、平台无关的 Modbus RTU 协议栈（纯 C），同时支持 **主机（Master）** 与 **从机（Slave）** 两种角色，适用于 MCU / RTOS / 裸机。

---

## 目录

- [1. 概述](#1-概述)
- [2. 设计理念](#2-设计理念)
- [3. 目录结构](#3-目录结构)
- [4. 快速开始](#4-快速开始)
- [5. 配置说明](#5-配置说明)
- [6. 数据模型](#6-数据模型)
- [7. 帧格式与 CRC](#7-帧格式与-crc)
- [8. 功能码详解](#8-功能码详解)
- [9. API 参考](#9-api-参考)
- [10. 错误码与异常码](#10-错误码与异常码)
- [11. IAP 升级协议](#11-iap-升级协议)
- [12. 移植指南](#12-移植指南)
- [13. 内存与缓冲区设计](#13-内存与缓冲区设计)
- [14. 注意事项与限制](#14-注意事项与限制)

---

## 1. 概述

本协议栈实现 Modbus RTU 帧处理（不含 ASCII 模式），核心职责是：

- **从机**：校验收到的请求帧，按寄存器映射读出/写入数据，组装并发送应答帧。
- **主机**：组装并发送请求帧，校验响应帧，通过回调把结果交给上层，并由 `tick` 驱动超时。

它**不负责**物理层：无定时器、无串口、无收发缓冲管理。收发通过一个发送回调 + 上层喂帧的方式与任意物理链路（UART / RS-485 等）解耦。

---

## 2. 设计理念

1. **纯帧处理库**：只做「帧的组装与解析」，定时器 / 串口 / 收发缓冲全部由上层实现。
2. **角色彻底解耦**：Slave / Master 各有独立头文件与源文件，互不依赖，可单独编译。
3. **宏开关裁剪**：功能码与角色都通过 `modbus_cfg.h` 里的宏开关控制，按需裁剪、控制体积。
4. **数据绑定双模**：从机数据支持「连续数组」与「回调（getter/setter）」两种方式，**回调优先**。
5. **收发解耦**：发送经 `mb_send_func_t` 回调接入底层；接收由上层把完整帧喂给 `mb_*_rx_frame`。
6. **主机单请求模型**：一次只挂一个未完成请求，忙时返回 `MB_ERR_BUSY`，`tick` 驱动超时。

---

## 3. 目录结构

```
modbus/
├── include/modbus/
│   ├── modbus_cfg.h     # 编译配置（角色 / 功能码 / 帧缓冲 / 数量上限）
│   ├── modbus_common.h  # 公共：功能码 / 异常码 / 错误码 / mb_base / 发送回调 / CRC16
│   ├── modbus_slave.h   # 从机：寄存器映射表 / 段结构 / 从机接口
│   ├── modbus_master.h  # 主机：请求结构 / 主机接口
│   └── modbus_iap.h     # IAP 升级：子命令 / 状态码 / 回调类型
├── src/
│   ├── modbus_common.c  # CRC16 + mb_base_init
│   ├── modbus_slave.c   # 从机逻辑
│   └── modbus_master.c  # 主机逻辑
├── examples/
│   ├── platform.h       # 示例适配层（仅用于编译 / 回环验证）
│   └── iap_demo.c       # IAP 内存回环演示
└── docs/                # 本文档
```

---

## 4. 快速开始

### 4.1 从机（Slave）

```c
#include "modbus/modbus_slave.h"

static uint16_t holding_regs[8];                 // 连续数组

static enum mb_err_t on_read_ver(uint16_t addr, uint16_t *out)
{
    switch (addr) {
    case 0x0000: *out = dev.serial_num; return MB_OK;   // 非连续 / 实时量，用回调
    case 0x0001: *out = adc_read();      return MB_OK;
    default:     return MB_ERR_ADDR;
    }
}

int main(void)
{
    struct mb_slave_handle self = {0};
    struct mb_reg_map map = {0};

    map.holding[0].start_addr = 0x0000;
    map.holding[0].num        = 8;
    map.holding[0].data       = holding_regs;
    map.holding_num           = 1;

    mb_slave_init(&self, 1, &map);       // 从机地址 1（内部清零句柄）
    self.base.send = my_send;            // 发送回调，须在 init 之后设置

    while (1) {
        uint8_t buf[128];
        uint8_t len = uart_rx_frame(buf);   // 上层攒满一帧
        if (len > 0)
            mb_slave_rx_frame(&self, buf, len);
    }
}
```

### 4.2 主机（Master）

```c
#include "modbus/modbus_master.h"

static struct mb_master_handle self;

static void on_rsp(enum mb_err_t err, const uint8_t *raw, uint8_t len, void *arg)
{
    if (err == MB_OK) {
        /* raw[3..] 为数据区（读寄存器：每寄存器 2 字节，高字节在前）*/
    } else if (err == MB_ERR_TIMEOUT) {
        /* 超时 */
    }
}

int main(void)
{
    mb_master_init(&self, my_send, NULL);

    /* 读从机 1 的保持寄存器 0x0000 起 2 个，超时 100 tick */
    mb_master_read(&self, 1, MB_FC_READ_HOLDING_REGS, 0x0000, 2, 100, on_rsp, NULL);

    while (1) {
        uint8_t buf[128];
        uint8_t len = uart_rx_frame(buf);
        if (len > 0)
            mb_master_rx_frame(&self, buf, len);

        mb_master_tick(&self);      // 周期调用，驱动超时
        delay_ms(1);
    }
}
```

---

## 5. 配置说明

所有开关集中在 `include/modbus/modbus_cfg.h`。

### 5.1 角色开关

| 宏               | 说明         | 默认 |
| :--------------- | :----------- | :---: |
| `MB_SLAVE_EN`  | 启用从机角色 | `1` |
| `MB_MASTER_EN` | 启用主机角色 | `1` |

### 5.2 功能码开关

| 宏             | 功能码 | 说明               |         默认         |
| :------------- | :----: | :----------------- | :-------------------: |
| `MB_FC01_EN` |  0x01  | 读线圈             |         `0`         |
| `MB_FC02_EN` |  0x02  | 读离散输入         |         `0`         |
| `MB_FC03_EN` |  0x03  | 读保持寄存器       |         `1`         |
| `MB_FC04_EN` |  0x04  | 读输入寄存器       |         `0`         |
| `MB_FC05_EN` |  0x05  | 写单线圈           |         `0`         |
| `MB_FC06_EN` |  0x06  | 写单寄存器         |         `0`         |
| `MB_FC0F_EN` |  0x0F  | 写多线圈           |         `0`         |
| `MB_FC10_EN` |  0x10  | 写多寄存器         |         `1`         |
| `MB_FC41_EN` |  0x41  | IAP 升级（自定义） | `1`（本工程已启用） |

### 5.3 帧缓冲与数量上限

| 宏                     | 说明                                                |   默认   |
| :--------------------- | :-------------------------------------------------- | :------: |
| `MB_TX_FRAME_SIZE`   | 发送帧缓冲大小（字节）                              | `256` |
| `MB_IAP_MAX_DATA`    | IAP 单块数据字节数（显式 128，2 块攒 1 页 flash 256） | `128` |
| `MB_MAX_READ_COILS`  | 单次读线圈 / 离散输入数量上限                       | `2000` |
| `MB_MAX_READ_REGS`   | 单次读寄存器数量上限                                |  `63`  |
| `MB_MAX_WRITE_COILS` | 单次写线圈数量上限                                  | `1968` |
| `MB_MAX_WRITE_REGS`  | 单次写寄存器数量上限                                |  `50`  |

> ⚠️ 注意：`MB_MAX_READ_COILS = 2000` 需要 `255` 字节的发送帧，当前 `MB_TX_FRAME_SIZE = 256` 刚好容纳。若增大 `MB_MAX_READ_COILS`，请同步调大 `MB_TX_FRAME_SIZE`。

---

## 6. 数据模型

### 6.1 段结构

从机数据空间由「段」（segment）描述，每个段支持**连续数组**或**回调**两种绑定方式，**回调优先**：

```c
/* 位段：每字节存 1 个线圈 / 离散输入（0 或 1）*/
struct mb_bit_seg {
    uint16_t start_addr;
    uint16_t num;
    uint8_t *data;                /* 连续数组（可选）*/
    mb_bit_read_cb_t  on_read;    /* 回调读（可选）*/
    mb_bit_write_cb_t on_write;   /* 回调写（可选）*/
};

/* 寄存器段 */
struct mb_reg_seg {
    uint16_t start_addr;
    uint16_t num;
    uint16_t *data;               /* 连续数组（可选）*/
    mb_reg_read_cb_t  on_read;    /* 回调读（可选）*/
    mb_reg_write_cb_t on_write;   /* 回调写（可选）*/
};
```

### 6.2 回调签名

回调返回 `MB_OK` 表示成功；返回 `MB_ERR_ADDR` 表示无此地址（协议栈转异常 `ILLEGAL_DATA_ADDRESS`）；返回其他错误码则转异常 `SLAVE_DEVICE_FAILURE`：

```c
typedef enum mb_err_t (*mb_bit_read_cb_t) (uint16_t addr, uint8_t  *out_val);
typedef enum mb_err_t (*mb_bit_write_cb_t)(uint16_t addr, uint8_t   val);
typedef enum mb_err_t (*mb_reg_read_cb_t) (uint16_t addr, uint16_t *out_val);
typedef enum mb_err_t (*mb_reg_write_cb_t)(uint16_t addr, uint16_t  val);
```

### 6.3 寄存器映射表

```c
struct mb_reg_map {
    struct mb_bit_seg coils[MB_COIL_SEGS];        uint8_t coils_num;
    struct mb_bit_seg discrete[MB_DISCRETE_SEGS]; uint8_t discrete_num;
    struct mb_reg_seg holding[MB_HOLDING_SEGS];   uint8_t holding_num;
    struct mb_reg_seg input[MB_INPUT_SEGS];       uint8_t input_num;
};
```

- 段数量由 `MB_COIL_SEGS` / `MB_DISCRETE_SEGS` / `MB_HOLDING_SEGS` / `MB_INPUT_SEGS` 决定（本工程 `MB_HOLDING_SEGS = 4`，其余为 `1`），可按需扩充。
- `mb_slave_init` 会**拷贝** `reg_map` 结构（含 `data` 指针与回调指针）到句柄内。

---

## 7. 帧格式与 CRC

### 7.1 RTU 帧

| 字段     |  长度  | 说明                             |
| :------- | :----: | :------------------------------- |
| 从站地址 | 1 字节 | `1–247` 单播，`0` 广播      |
| 功能码   | 1 字节 | 见功能码详解                     |
| 数据     | N 字节 | 依功能码而定                     |
| CRC      | 2 字节 | **低字节在前**，高字节在后 |

### 7.2 CRC16

- 多项式：`0xA001`（即 `0x8005` 位反转）
- 初值：`0xFFFF`
- 覆盖范围：帧中除 CRC 本身以外的所有字节

相关函数（已公开于 `modbus_common.h`）：

```c
uint16_t mb_crc16(const uint8_t *buf, uint8_t len);   // 计算
void     mb_append_crc(uint8_t *frame, uint8_t len);  // 帧尾追加 CRC
uint8_t  mb_check_crc(const uint8_t *frame, uint8_t len); // 校验
```

---

## 8. 功能码详解

从机收到帧后先验 CRC、再比对地址（`slave_addr` 或广播 `0x00`），最后按功能码分派。广播 `0x00` 的请求会被**执行但不应答**（符合 Modbus 规范，避免多从机同时回帧冲突）。

### 8.1 读线圈 0x01 / 读离散输入 0x02

- 请求：`addr, fc, 起始地址(2), 数量(2)`
- 响应：`addr, fc, 字节数(1), 位数据(N), crc`，位按 LSB 优先打包
- 数量 0 或超过 `MB_MAX_READ_COILS` → 异常 `ILLEGAL_DATA_VALUE`

### 8.2 读保持寄存器 0x03 / 读输入寄存器 0x04

- 请求：`addr, fc, 起始地址(2), 数量(2)`
- 响应：`addr, fc, 字节数(1), 寄存器数据(2×N), crc`，每寄存器 2 字节**高字节在前**
- 数量 0 或超过 `MB_MAX_READ_REGS` → 异常 `ILLEGAL_DATA_VALUE`

### 8.3 写单线圈 0x05

- 请求：`addr, 0x05, 地址(2), 值(2)`，值 `0xFF00` = ON、`0x0000` = OFF
- 响应：回显请求（`addr, 0x05, 地址(2), 值(2)`）
- 值既非 `0x0000` 也非 `0xFF00` → 异常 `ILLEGAL_DATA_VALUE`

### 8.4 写单寄存器 0x06

- 请求：`addr, 0x06, 地址(2), 值(2)`
- 响应：回显请求

### 8.5 写多线圈 0x0F

- 请求：`addr, 0x0F, 起始地址(2), 数量(2), 字节数(1), 位数据(N)`
- 响应：`addr, 0x0F, 起始地址(2), 数量(2)`

### 8.6 写多寄存器 0x10

- 请求：`addr, 0x10, 起始地址(2), 数量(2), 字节数(1), 寄存器数据(2×N)`
- 响应：`addr, 0x10, 起始地址(2), 数量(2)`

### 8.7 IAP 升级 0x41

见 [11. IAP 升级协议](#11-iap-升级协议)。

---

## 9. API 参考

### 9.1 公共层（`modbus_common.h`）

| 函数 / 类型                                                      | 说明                                                       |
| :--------------------------------------------------------------- | :--------------------------------------------------------- |
| `mb_send_func_t`                                               | 发送回调：`uint8_t (*)(const uint8_t *buf, uint8_t len, void *ctx)` |
| `struct mb_base`                                               | 基类：`send` 回调 + `tx_frame` 发送缓冲                |
| `void mb_base_init(struct mb_base *base, mb_send_func_t send, void *ctx)` | 初始化基类（`memset` 后设置 `send`）                   |
| `uint16_t mb_crc16(const uint8_t *buf, uint8_t len)`           | 计算 CRC16                                                 |
| `void mb_append_crc(uint8_t *frame, uint8_t len)`              | 帧尾追加 CRC                                               |
| `uint8_t mb_check_crc(const uint8_t *frame, uint8_t len)`      | 校验帧 CRC                                                 |

### 9.2 从机层（`modbus_slave.h`）

| 函数                                               | 说明                                           |
| :------------------------------------------------- | :--------------------------------------------- |
| `void mb_slave_init(handle, addr, reg_map)`      | 初始化：从站地址、寄存器映射表（拷贝）         |
| `uint8_t mb_slave_rx_frame(handle, p_data, len)` | 处理一帧：验 CRC → 比对地址 → 分派处理并应答 |

从机侧 IAP 数据接收通过句柄内的 `iap` 回调完成（见 [11](#11-iap-升级协议)）。

### 9.3 主机层（`modbus_master.h`）

| 函数                                                                                          | 说明                  |
| :-------------------------------------------------------------------------------------------- | :-------------------- |
| `void mb_master_init(handle, send, ctx)`                                                         | 初始化主机            |
| `void mb_master_rx_frame(handle, p_data, len)`                                              | 处理收到的响应帧      |
| `void mb_master_tick(handle)`                                                               | 周期调用，驱动超时    |
| `mb_master_read(handle, addr, fc, reg_addr, reg_num, timeout, on_rsp, arg)`                 | 读请求（FC01–04）    |
| `mb_master_write_single(handle, addr, fc, addr, value, timeout, on_rsp, arg)`               | 写单个（FC05 / FC06） |
| `mb_master_write_multi(handle, addr, fc, start, num, data, data_len, timeout, on_rsp, arg)` | 写多个（FC0F / FC10） |
| `mb_master_iap_start(handle, slave, total_size, fw_crc32, timeout, on_rsp, arg)`            | IAP：开始升级         |
| `mb_master_iap_write(handle, slave, block_no, data, len, timeout, on_rsp, arg)`             | IAP：写数据块         |
| `mb_master_iap_end(handle, slave, total_blocks, timeout, on_rsp, arg)`                      | IAP：结束升级         |
| `mb_master_iap_status(handle, slave, timeout, on_rsp, arg)`                                 | IAP：查询状态         |

响应回调签名：

```c
typedef void (*mb_master_rsp_cb_t)(enum mb_err_t err, const uint8_t *raw, uint8_t len, void *arg);
```

- 若 `err == MB_OK`，`raw` 指向完整响应帧（含 CRC），`raw[0]` 地址、`raw[1]` 功能码、`raw[2..]` 数据区。
- 若从机返回异常，`err == MB_ERR_FC`；超时则 `err == MB_ERR_TIMEOUT`（此时 `raw == NULL`、`len == 0`）。

---

## 10. 错误码与异常码

### 10.1 错误码（`enum mb_err_t`）

|   值   | 名称               | 含义                      |
| :----: | :----------------- | :------------------------ |
| `0` | `MB_OK`          | 成功                      |
| `-1` | `MB_ERR_CRC`     | CRC 校验失败              |
| `-2` | `MB_ERR_ADDR`    | 地址不匹配 / 无此地址     |
| `-3` | `MB_ERR_FC`      | 功能码错误 / 从机返回异常 |
| `-4` | `MB_ERR_RANGE`   | 寄存器范围越界            |
| `-5` | `MB_ERR_LEN`     | 帧长度错误                |
| `-6` | `MB_ERR_BUSY`    | 主机忙（上一请求未完成）  |
| `-7` | `MB_ERR_TIMEOUT` | 响应超时                  |

### 10.2 Modbus 异常码

从机在以下情况返回异常帧（`addr, fc|0x80, 异常码, crc`）：

|    值    | 名称                           | 触发场景                           |
| :------: | :----------------------------- | :--------------------------------- |
| `0x01` | `MB_EX_ILLEGAL_FUNCTION`     | 功能码未启用 / 不支持              |
| `0x02` | `MB_EX_ILLEGAL_DATA_ADDRESS` | 地址越界 / 回调返回`MB_ERR_ADDR` |
| `0x03` | `MB_EX_ILLEGAL_DATA_VALUE`   | 数量或值非法                       |
| `0x04` | `MB_EX_SLAVE_DEVICE_FAILURE` | 从机设备故障（回调返回其他错误）               |

---

## 11. IAP 升级协议

IAP 通过自定义功能码 `0x41` 传输固件数据，**只负责「接数据」**：把主机发来的字节流按块、按序交给用户回调。flash 分区 / 跳转 / 激活回滚由用户在回调中自行处理。

### 11.1 帧格式

| 子命令 |    值    | 请求                                           | 响应（status 位于`raw[3]`）            |
| :----- | :------: | :--------------------------------------------- | :--------------------------------------- |
| START  | `0x01` | `addr,0x41,0x01,total_size(4B),fw_crc32(4B)` | `addr,0x41,0x01,status`                |
| DATA   | `0x02` | `addr,0x41,0x02,block_no(2B),data(N)`        | `addr,0x41,0x02,status`                |
| END    | `0x03` | `addr,0x41,0x03,total_blocks(2B)`            | `addr,0x41,0x03,status`                |
| STATUS | `0x04` | `addr,0x41,0x04`                             | `addr,0x41,0x04,status,next_block(2B)` |

- `block_no` 从 0 起**严格连续**，否则从机回 `MB_IAP_BAD_BLOCK`。
- `STATUS` 返回期望的下一个块号，主机可据此断点续传。
- 单块 `MB_IAP_MAX_DATA` = 128 字节（2 块攒 1 页 flash 256，末页残块在 END 时 flush）。
- 响应帧格式：`raw[0]` 地址、`raw[1] = 0x41`、`raw[2]` 回显子命令、`raw[3]` 状态码。
- `fw_crc32` 为整个固件镜像的 CRC32（标准 CRC-32，多项式 `0xEDB88320`，初值/终值 `0xFFFFFFFF`），大端。START 时记录到 `iap.expected_crc32` 并经 `on_start(total_size, crc32)` 透传；库不自动校验，比对请在 `on_end` 回调中完成。

### 11.2 状态码（`enum mb_iap_status_t`）

|    值    | 名称                  | 含义                   |
| :------: | :-------------------- | :--------------------- |
| `0x00` | `MB_IAP_OK`         | 成功                   |
| `0x01` | `MB_IAP_BAD_BLOCK`  | 块号不连续             |
| `0x02` | `MB_IAP_BAD_LEN`    | 数据长度错误           |
| `0x03` | `MB_IAP_NOT_ACTIVE` | 未 START 就 DATA / END |
| `0x04` | `MB_IAP_INTERNAL`   | 用户回调返回错误 / 缺少 on_data 回调 |

### 11.3 从机（接数据）

```c
static enum mb_err_t iap_on_data(uint16_t block_no, const uint8_t *data, uint8_t len)
{
    uint32_t off = (uint32_t)block_no * MB_IAP_MAX_DATA;
    flash_write(off, data, len);   // 用户自行写 flash
    return MB_OK;
}

/* 初始化后注册回调 */
self.iap.on_data  = iap_on_data;   // 必须
self.iap.on_start = iap_on_start;  // 可选（START 时调用，携带 total_size 与 crc32）
self.iap.on_end   = iap_on_end;    // 可选（END 时调用）
```

> ⚠️ `on_data` 的 `data` 指针指向**上层传入的接收帧内存**，协议栈不复制。回调内必须**同步**把数据复制走或写入 flash，回调返回后该内存可能被下次接收覆盖。

### 11.4 主机（发送）

```c
static void on_rsp(enum mb_err_t err, const uint8_t *raw, uint8_t len, void *arg)
{
    if (err == MB_OK && raw[3] == MB_IAP_OK) {
        /* 发下一块 mb_master_iap_write(...)，或结束 mb_master_iap_end(...) */
    }
}

/* 开始升级；逐块发送（单请求模型：等上一块响应确认后再发下一块）*/
mb_master_iap_start(&self, slave, total_size, fw_crc32, 100, on_rsp, NULL);
```

完整回环示例见 `examples/iap_demo.c`（含正常流程与块号跳变负例）。

---

## 12. 移植指南

### 12.1 提供 `platform.h`

协议栈依赖 `platform.h` —— 一个**适配用户系统的头文件**，用于放置用户按自身平台 / 工具链适配的基础类型与函数（本仓库不含该文件，`examples/platform.h` 仅作示例）：

```c
#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <stdint.h>   // uint8_t / uint16_t / uint32_t
#include <string.h>   // memset / memcpy

#endif
```

### 12.2 提供发送回调

```c
uint8_t my_send(const uint8_t *buf, uint8_t len, void *ctx)
{
    (void)ctx;   /* 单实例可忽略，多实例用 ctx 找回实例 */
    // 将 buf[0..len-1] 写入 UART / 485 发送缓冲区（同步或启动 DMA）
    return uart_tx(buf, len);
}
```

### 12.3 接收侧

将从串口收到的字节积累成**完整帧（含 CRC）**后，调用 `mb_slave_rx_frame()` / `mb_master_rx_frame()`。分帧通常依赖 RTU 的 3.5 字符静默间隔，由上层定时器实现。

### 12.4 主机超时驱动

主机需周期调用 `mb_master_tick()`，其内部计数器以「tick」为单位（如 1 tick = 1 ms），超时阈值由各请求的 `timeout_tick` 参数指定。

---

## 13. 内存与缓冲区设计

| 项                   | 说明                                                                                                    |
| :------------------- | :------------------------------------------------------------------------------------------------------ |
| 发送缓冲`tx_frame` | 句柄内嵌，`MB_TX_FRAME_SIZE` 字节。从机应答与主机请求共用此工作区拼帧，拼完即 `send` 发出           |
| 接收缓冲             | **协议栈不内置**。接收数据由上层缓冲，`rx_frame` 直接读上层传入的 `p_data` 指针，不复制不缓存 |

**设计取舍**：

- 拼帧必须有一块连续内存（CRC 在帧尾，需拼完整帧再算 CRC），故 `tx_frame` 内嵌于句柄——**不占栈**（裸机栈紧张）、**生命周期与 handle 一致**（`send` 用 DMA 异步也安全）、**大小显式可控**。
- 接收零拷贝：IAP 等大数据场景，数据全程在用户侧，协议栈只在回调里把指针交出，不引入额外的接收缓冲拷贝。

---

## 14. 注意事项与限制

- 协议栈为**纯帧处理库**，不含定时器 / 串口 / 收发缓冲管理，需由上层实现。
- 从机按 **RTU** 处理（无 ASCII 模式），接收帧需为**完整帧（含 CRC）**。
- 主机为**单请求**模型：上一请求未完成（`MB_ERR_BUSY`）时需等待响应或超时。
- 同一段同时配置 `data` 与 `on_read` / `on_write` 时，**回调优先**。
- IAP（`0x41`）为自定义功能码，需主从双方约定一致；从机只「接数据」，存储 / 跳转 / 回滚由用户在回调中实现。
- `mb_slave_init` 会先 `memset` 清零整个句柄，再设置 `slave_addr` 并拷贝 `reg_map`，因此发送回调 `base.send` 需在 `mb_slave_init` 之后赋值。
- `MB_TX_FRAME_SIZE` 需与最大发送帧长匹配（尤其启用大数量读线圈时），否则存在溢出风险。
