# Modbus RTU 协议栈

> 📖 完整说明见 [用户指南](docs/user-guide.md)。

一个轻量级、平台无关的 **Modbus RTU** 协议栈（纯 C 实现），同时支持 **主机（Master）** 与 **从机（Slave）** 两种角色，适用于各类嵌入式平台（MCU / RTOS / 裸机）。

- 零外部依赖（仅需用户提供 `platform.h` 中的基础类型与 `memset`）
- Master / Slave 彻底解耦，各自独立头文件与源文件，可单独编译
- 通过宏开关按需裁剪功能码与角色，编译体积可控
- 从机数据绑定支持「连续数组」与「回调（getter/setter）」两种方式，兼顾简单场景与非连续内存

---

## 版本号

库版本通过 `modbus_common.h` 暴露（编译期可见）：

| 宏 | 说明 |
|---|---|
| `MB_RTU_VERSION_MAJOR` / `_MINOR` / `_PATCH` | 三段版本号（当前 `1.0.0`） |
| `MB_RTU_VERSION_STRING` | 版本字符串 `"1.0.0"` |
| `MB_RTU_VERSION_NUM` | 打包数值 `0x010000`，用于版本比较 |

## 目录结构

```
modbus/
├── include/
│   └── modbus/
│       ├── modbus_cfg.h     # 编译配置（角色 / 功能码 / 帧缓冲 / 数量上限）
│       ├── modbus_common.h  # 公共：功能码 / 异常码 / 错误码 / mb_base / 发送回调 / CRC16
│       ├── modbus_slave.h   # 从机：寄存器映射表 / 段结构 / 从机接口
│       ├── modbus_master.h  # 主机：请求结构 / 主机接口
│       └── modbus_iap.h     # IAP 升级：子命令 / 状态码 / 回调类型
└── src/
    ├── modbus_common.c      # CRC16 + mb_base_init
    ├── modbus_slave.c       # 从机逻辑
    └── modbus_master.c      # 主机逻辑
```

---

## 特性

- **角色解耦**：Slave / Master 各一套头文件 + 源文件，互不依赖，按需编译
- **标准功能码**：支持 Modbus 常用功能码，可按需裁剪
- **RTU 帧**：自动计算并校验 CRC16（多项式 `0xA001`）
- **异常应答**：从机返回标准 Modbus 异常码
- **从机**：分段寄存器映射（线圈 / 离散输入 / 保持寄存器 / 输入寄存器）
- **主机**：异步请求、忙检测、超时处理、响应回调
- **收发解耦**：通过 `mb_send_func_t` 发送回调接入任意底层（UART、485 等）

---

## 支持的功能码

| 功能码 | 名称 | 说明 | 默认配置 |
| :---: | :--- | :--- | :---: |
| `0x01` | Read Coils | 读线圈 | 关 |
| `0x02` | Read Discrete Inputs | 读离散输入 | 关 |
| `0x03` | Read Holding Registers | 读保持寄存器 | **开** |
| `0x04` | Read Input Registers | 读输入寄存器 | 关 |
| `0x05` | Write Single Coil | 写单线圈 | 关 |
| `0x06` | Write Single Register | 写单寄存器 | 关 |
| `0x0F` | Write Multiple Coils | 写多线圈 | 关 |
| `0x10` | Write Multiple Registers | 写多寄存器 | **开** |
| `0x41` | IAP（自定义） | 在线升级数据传输 | **开** |

> 各功能码的开关在 `modbus_cfg.h` 中通过 `MB_FCxx_EN` 宏配置。

---

## 配置说明（`modbus_cfg.h`）

| 宏 | 说明 | 默认值 |
| :--- | :--- | :--- |
| `MB_SLAVE_EN` | 启用从机角色 | `1` |
| `MB_MASTER_EN` | 启用主机角色 | `1` |
| `MB_FC01_EN` ~ `MB_FC10_EN` | 各功能码开关 | 见上表 |
| `MB_FC41_EN` | IAP 升级功能码开关 | `1`（本工程已启用） |
| `MB_TX_FRAME_SIZE` | 发送帧缓冲大小（字节） | `256` |
| `MB_IAP_MAX_DATA` | IAP 单块数据字节数（显式 128，2 块攒 1 页 flash 256） | `128` |
| `MB_MAX_READ_COILS` | 单次读线圈 / 离散输入数量上限 | `2000` |
| `MB_MAX_READ_REGS` | 单次读寄存器数量上限 | `63` |
| `MB_MAX_WRITE_COILS` | 单次写线圈数量上限 | `1968` |
| `MB_MAX_WRITE_REGS` | 单次写寄存器数量上限 | `50` |

---

## 移植说明

协议栈依赖 `platform.h` —— 这是一个**适配用户系统的头文件**，用于放置用户按自身平台 / 工具链适配的基础类型（如 `uint8_t` / `uint16_t`）与 `memset`。该文件需在你的平台项目中提供（**本仓库不包含此文件**）：

```c
// platform.h（示例）
#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <stdint.h>   // uint8_t / uint16_t
#include <string.h>   // memset

#endif
```

同时需要提供一个 **发送回调**，把协议栈生成的帧发到物理链路（如 UART / RS-485）：

```c
uint8_t my_send(const uint8_t *buf, uint8_t len, void *ctx)
{
    (void)ctx;   /* 单实例可忽略，多实例用 ctx 找回实例 */
    // 将 buf[0..len-1] 写入 UART / 485 发送缓冲区
    return uart_tx(buf, len);
}
```

接收侧：将从串口收到的数据（完整帧）调用 `mb_slave_rx_frame()` / `mb_master_rx_frame()` 交给协议栈处理。

---

## 数据模型

### 段结构

从机的数据空间由「段」（segment）描述。每个段支持两种绑定方式，**回调优先，其次连续数组**：

- **连续数组**（`data`）：数据在 RAM 中连续，直接绑定指针即可
- **回调**（`on_read` / `on_write`）：数据非连续、映射到外设、或需实时计算时使用

```c
/* 位段：每字节存 1 个线圈/离散输入（0 或 1）*/
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

### 回调签名

回调返回 `MB_OK` 表示成功，`MB_ERR_ADDR` 表示无此地址（协议栈将转为异常 `ILLEGAL_DATA_ADDRESS`）：

```c
typedef enum mb_err_t (*mb_bit_read_cb_t) (uint16_t addr, uint8_t  *out_val);
typedef enum mb_err_t (*mb_bit_write_cb_t)(uint16_t addr, uint8_t   val);
typedef enum mb_err_t (*mb_reg_read_cb_t) (uint16_t addr, uint16_t *out_val);
typedef enum mb_err_t (*mb_reg_write_cb_t)(uint16_t addr, uint16_t  val);
```

### 寄存器映射表

```c
struct mb_reg_map {
    struct mb_bit_seg coils[MB_COIL_SEGS];        uint8_t coils_num;
    struct mb_bit_seg discrete[MB_DISCRETE_SEGS]; uint8_t discrete_num;
    struct mb_reg_seg holding[MB_HOLDING_SEGS];   uint8_t holding_num;
    struct mb_reg_seg input[MB_INPUT_SEGS];       uint8_t input_num;
};
```

> 段数量由 `MB_COIL_SEGS` / `MB_DISCRETE_SEGS` / `MB_HOLDING_SEGS` / `MB_INPUT_SEGS` 宏决定（本工程 `MB_HOLDING_SEGS = 4`，其余为 `1`），可按需扩充。

---

## 快速开始

### 1. 从机（Slave）

```c
#include "modbus/modbus_slave.h"

/* 方式一：连续数组 */
static uint16_t holding_regs[8];

/* 方式二：非连续 / 自定义数据，用回调 */
static enum mb_err_t on_read_ver(uint16_t addr, uint16_t *out)
{
    switch (addr) {
    case 0x0000: *out = dev.serial_num; return MB_OK;  // 散落在结构体字段
    case 0x0001: *out = adc_read();      return MB_OK;  // 实时量
    default:     return MB_ERR_ADDR;
    }
}

int main(void)
{
    struct mb_slave_handle self;
    struct mb_reg_map map = {0};

    /* 段 0：地址 0x0000 起 8 个寄存器，绑定连续数组 */
    map.holding[0].start_addr = 0x0000;
    map.holding[0].num        = 8;
    map.holding[0].data       = holding_regs;
    map.holding_num           = 1;

    mb_slave_init(&self, 1, &map);   // 从机地址 1（内部清零句柄）
    self.base.send = my_send;        // 发送回调，须在 init 之后设置

    while (1) {
        uint8_t buf[128];
        uint8_t len = uart_rx_frame(buf);
        if (len > 0)
            mb_slave_rx_frame(&self, buf, len);   // 校验 CRC 并应答
    }
}
```

### 2. 主机（Master）

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

        mb_master_tick(&self);        // 周期调用，驱动超时
        delay_ms(1);
    }
}
```

---

## IAP 升级（功能码 0x41）

IAP 通过自定义功能码 `0x41` 传输固件数据，**只负责「接数据」**：把主机发来的字节流按块、按序交给用户回调，不关心 flash 分区 / 跳转 / 激活回滚（这些由用户在回调里自行处理）。

### 协议

| 子命令 | 值 | 请求 | 响应（status 位于 `raw[3]`） |
| :--- | :---: | :--- | :--- |
| START | `0x01` | `addr,0x41,0x01,total_size(4B),fw_crc32(4B)` | `addr,0x41,0x01,status` |
| DATA | `0x02` | `addr,0x41,0x02,block_no(2B),data(N)` | `addr,0x41,0x02,status` |
| END | `0x03` | `addr,0x41,0x03,total_blocks(2B)` | `addr,0x41,0x03,status` |
| STATUS | `0x04` | `addr,0x41,0x04` | `addr,0x41,0x04,status,next_block(2B)` |

- `block_no` 从 0 起严格连续，否则从机回 `MB_IAP_BAD_BLOCK`；`STATUS` 可查到期望块号，用于断点续传。
- 单块 `MB_IAP_MAX_DATA` = 128 字节（2 块攒 1 页 flash 256，末页残块在 END 时 flush）。
- `fw_crc32` 为整个固件镜像的 CRC32（标准 CRC-32，多项式 `0xEDB88320`，初值/终值 `0xFFFFFFFF`），大端。START 时记录到 `iap.expected_crc32` 并经 `on_start(total_size, crc32)` 透传；库不自动校验，比对请在 `on_end` 回调中完成。

### 状态码

| 值 | 含义 |
| :---: | :--- |
| `MB_IAP_OK` (0x00) | 成功 |
| `MB_IAP_BAD_BLOCK` (0x01) | 块号不连续 |
| `MB_IAP_BAD_LEN` (0x02) | 数据长度错误 |
| `MB_IAP_NOT_ACTIVE` (0x03) | 未 START 就 DATA/END |
| `MB_IAP_INTERNAL` (0x04) | 用户回调返回错误 / 缺少 on_data 回调 |

### 从机（接数据）

```c
static enum mb_err_t iap_on_data(uint16_t block_no, const uint8_t *data, uint8_t len)
{
    uint32_t off = (uint32_t)block_no * MB_IAP_MAX_DATA;
    flash_write(off, data, len);   // 用户自行写 flash
    return MB_OK;
}

/* 初始化后注册回调 */
self.iap.on_data  = iap_on_data;   // 必须
self.iap.on_start = iap_on_start;  // 可选
self.iap.on_end   = iap_on_end;    // 可选
```

### 主机（发送）

```c
static void on_rsp(enum mb_err_t err, const uint8_t *raw, uint8_t len, void *arg)
{
    if (err == MB_OK && raw[3] == MB_IAP_OK) {
        /* 发下一块 mb_master_iap_write(...) 或结束 mb_master_iap_end(...) */
    }
}

/* 开始升级，逐块发送（单请求模型：等上一块响应确认后再发下一块）*/
mb_master_iap_start(&self, slave, total_size, fw_crc32, 100, on_rsp, NULL);
```

> 完整回环示例见 `examples/iap_demo.c`（含正常流程与块号跳变负例）。

---

## API 参考

### 公共（`modbus_common.h`）

| 函数 | 说明 |
| :--- | :--- |
| `void mb_base_init(struct mb_base *base, mb_send_func_t send, void *ctx)` | 初始化基类，设置发送回调 |

### 从机（`modbus_slave.h`）

| 函数 | 说明 |
| :--- | :--- |
| `void mb_slave_init(handle, addr, reg_map)` | 初始化从机：地址、寄存器映射表 |
| `uint8_t mb_slave_rx_frame(handle, p_data, len)` | 处理一帧数据（校验 CRC 后应答） |

### 主机（`modbus_master.h`）

| 函数 | 说明 |
| :--- | :--- |
| `void mb_master_init(handle, send, ctx)` | 初始化主机 |
| `void mb_master_rx_frame(handle, p_data, len)` | 处理收到的响应帧 |
| `void mb_master_tick(handle)` | 周期调用，驱动超时 |
| `mb_master_read(handle, addr, fc, reg_addr, reg_num, timeout, on_rsp, arg)` | 读请求（FC01–04） |
| `mb_master_write_single(handle, addr, fc, addr, value, timeout, on_rsp, arg)` | 写单个（FC05 / FC06） |
| `mb_master_write_multi(handle, addr, fc, start, num, data, data_len, timeout, on_rsp, arg)` | 写多个（FC0F / FC10） |
| `mb_master_iap_start(handle, slave, total_size, fw_crc32, timeout, on_rsp, arg)` | IAP：开始升级 |
| `mb_master_iap_write(handle, slave, block_no, data, len, timeout, on_rsp, arg)` | IAP：写数据块 |
| `mb_master_iap_end(handle, slave, total_blocks, timeout, on_rsp, arg)` | IAP：结束升级 |
| `mb_master_iap_status(handle, slave, timeout, on_rsp, arg)` | IAP：查询状态 |

### 错误码（`enum mb_err_t`）

| 值 | 含义 |
| :--- | :--- |
| `MB_OK` | 成功 |
| `MB_ERR_CRC` | CRC 校验失败 |
| `MB_ERR_ADDR` | 地址不匹配 / 无此地址 |
| `MB_ERR_FC` | 功能码错误 / 从机返回异常 |
| `MB_ERR_RANGE` | 寄存器范围越界 |
| `MB_ERR_LEN` | 帧长度错误 |
| `MB_ERR_BUSY` | 主机忙（上一请求未完成） |
| `MB_ERR_TIMEOUT` | 响应超时 |

---

## 注意事项

- 该协议栈为 **纯帧处理库**，不含定时器 / 串口 / 收发缓冲管理，需由上层实现。
- 从机按 **RTU** 处理（无 ASCII 模式），接收帧需为完整帧（含 CRC）。
- 广播地址 `0x00` 的请求从机会执行但**不应答**（符合 Modbus 规范）。
- 主机为 **单请求** 模型：上一次请求未完成（`MB_ERR_BUSY`）时需等待响应或超时。
- 同一段同时配置 `data` 与 `on_read` / `on_write` 时，**回调优先**。
- IAP（`0x41`）为自定义功能码，需主从双方约定一致；从机只「接数据」，存储 / 跳转 / 回滚由用户在回调中实现。
