# 03 — IAP 协议（MB-RTU FC41）

> 协议由 `third_party/MB-RTU` 的 `modbus_iap.h` / `modbus_slave.c` 实现，本文件是工程视角的规格与 bootloader 侧行为约定。

## 1. 帧格式（RTU）

功能码 `0x41`（自定义）。请求 / 响应均为 Modbus RTU 帧（addr + FC + 数据 + CRC16）。

| 子命令 | 值 | 请求数据区 | 响应数据区（status 在 `raw[3]`） |
| --- | --- | --- | --- |
| START | 0x01 | `total_size(4B) + fw_crc32(4B)` | `status` |
| DATA | 0x02 | `block_no(2B) + data(N)` | `status` |
| END | 0x03 | `total_blocks(2B)` | `status` |
| STATUS | 0x04 | （无） | `status + next_block(2B)` |

- `total_size` / `fw_crc32` / `block_no` / `total_blocks` 均为**大端**。
- `block_no` 从 0 起严格连续；不连续 → 从机回 `MB_IAP_BAD_BLOCK`。
- `STATUS` 返回期望的 `next_block`，用于断点续传。
- `fw_crc32`：整个固件镜像的 CRC32（标准 CRC-32，poly `0xEDB88320`，初值/终值 `0xFFFFFFFF`，大端）。

## 2. 状态码

| 值 | 含义 |
| --- | --- |
| 0x00 | 成功 |
| 0x01 | 块号不连续 |
| 0x02 | 数据长度错误 |
| 0x03 | 未 START 就 DATA/END |
| 0x04 | 用户回调返回错误 |

## 3. Bootloader 实现的回调

MB-RTU 从机侧把「接数据」完全交给用户回调，bootloader 实现三个：

```c
/* START：记录期望，准备擦 APP2 */
enum mb_err_t iap_on_start(uint32_t total_size, uint32_t crc32);

/* DATA：写 APP2（攒 2KB 页） */
enum mb_err_t iap_on_data(uint16_t block_no, const uint8_t *data, uint8_t len);

/* END：flush 末页 + CRC32 校验 + 置 APP2_READY + 触发 COPY/COMMIT */
enum mb_err_t iap_on_end(void);
```

注册：

```c
hdl.iap.on_start = iap_on_start;
hdl.iap.on_data  = iap_on_data;
hdl.iap.on_end   = iap_on_end;
```

## 4. 数据写入策略（攒 2 KB 页）

- 单块 `MB_IAP_MAX_DATA = 128` 字节。
- **2 KB 页 = 16 块**（16 × 128 B = 2048 B）。
- bootloader 维护一个 2 KB 页缓冲 + `app2_next_page`、`page_fill`：
  - 每收一块写入页缓冲；
  - 攒满 16 块 → 擦目标页 + 编程（128-bit 对齐，128 B 块 = 8 × 16 B，天然对齐）；
  - 末页残块在 `on_end` 时用 `0xFF` 补齐到 16 字节边界后擦写。

```text
block(128B) × 16  →  2 KB 页缓冲  →  擦 1 页 + 编程 1 页
```

## 5. CRC32 校验

- 算法：标准 CRC-32，poly `0xEDB88320`，初值 `0xFFFFFFFF`，输出异或 `0xFFFFFFFF`，大端。
- `on_end` 时对 APP2 整图（`total_size` 字节）计算 CRC32，与 `on_start` 记录的 `fw_crc32` 比对。
- 通过 → 置 `APP2_READY` + app2_size/app2_crc32 → 进入 COPY；
- 不通过 → 回 `MB_ERR`（从机回 `MB_IAP_INTERNAL`），停在 IAP 等重传。
- 需要新增 CRC32 工具（现有只有 Modbus CRC16）。

## 6. 升级时序

```text
上位机                              Bootloader
   │  START(total_size, crc32) ────────>│ 记录期望，准备擦 APP2
   │  DATA(0, 128B) ──────────────────>│ 写页缓冲
   │  DATA(1..14) ────────────────────>│ 攒页
   │  DATA(15) ───────────────────────>│ 满 16 块 → 擦页 + 编程
   │  ... 重复 ...                      │
   │  END(total_blocks) ───────────────>│ flush 末页 + CRC32 校验
   │                                    │ 通过 → COPY → COMMIT → 复位
   │  STATUS ──────────────────────────>│ 返回 next_block（会话内查询）
```

## 7. 上位机侧

上位机用 MB-RTU 的 master API 按顺序调用：

```c
mb_master_iap_start (&hdl, slave, total_size, fw_crc32, timeout, on_rsp, NULL);
mb_master_iap_write (&hdl, slave, block_no, data, len, timeout, on_rsp, NULL);  /* 循环 */
mb_master_iap_end   (&hdl, slave, total_blocks, timeout, on_rsp, NULL);
mb_master_iap_status(&hdl, slave, timeout, on_rsp, NULL);  /* 断点续传查询 */
```

单请求模型：上一块响应确认后再发下一块；`on_rsp` 里读 `raw[3]` 判断 status。

## 8. 状态寄存器（FC03 只读）

bootloader 作为 Modbus 从机，除 FC41 IAP 外，还暴露一组**只读状态寄存器**（保持寄存器，FC03），
让上位机随时查询 bootloader 状态——尤其**两个 APP 都无效**时，能确认「bootloader 空闲、等待下载」。

| 地址 | 名称 | 说明 |
| --- | --- | --- |
| 0x0000 | STATE | 状态机：0=IDLE, 1=RECEIVING, 2=VERIFYING, 3=COPYING, 4=COMMITTING, 5=DONE, 6=ERROR |
| 0x0001 | BOOT_REASON | 为何在 bootloader：0=正常(APP有效), 1=APP无效, 2=强制IAP, 3=无程序(空META/首次生产) |
| 0x0002 | APP_VALID | APP 运行槽是否有效 0/1 |
| 0x0003 | DL_VALID | 下载槽是否有完整镜像 0/1 |
| 0x0004 | ERROR | 最近错误码：0=无, 1=CRC失败, 2=FLASH失败, 3=范围越界, 4=状态错误 |

> 典型场景（APP1/APP2 都无效）：`STATE=0, BOOT_REASON=1, APP_VALID=0, DL_VALID=0, ERROR=0`
> → 上位机可判定「bootloader 空闲，等待下载完整固件」。

对应库 API：`bl_get_status()` 返回 `bl_status_t`，Modbus 适配器把字段映射到上面 5 个寄存器。
