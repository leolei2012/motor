# 02 — 内存布局与 META 页

## 1. Flash 参数（以 HAL 头为准）

| 项 | 值 |
| --- | --- |
| 总容量 | 512 KB = 256 页 × **2 KB**（`FLASH_PAGE_SIZE = 0x800`） |
| 页擦除粒度 | 2 KB（`FLASH_PAGE_NB = 256`，单 bank） |
| 编程宽度（单 bank） | 128-bit（16 字节，需 16 字节对齐） |
| 基址 | `FLASH_BASE = 0x08000000` |

> 保持单 bank（`DBANK=0`，默认）。双 bank 会变成 64-bit 编程宽度且需改 option byte，本方案不用。

## 2. Flash 分区

| 区域 | 起始 | 结束（含） | 大小 | 页数 | 说明 |
| --- | --- | --- | --- | --- | --- |
| Bootloader | 0x0800_0000 | 0x0800_7FFF | 32 KB | 16 | 固定入口，**永不擦** |
| APP1 | 0x0800_8000 | 0x0804_17FF | 230 KB | 115 | 运行槽，固件唯一链接地址 |
| APP2 | 0x0804_1800 | 0x0807_AFFF | 230 KB | 115 | 下载缓冲，只存字节 |
| NV Storage | 0x0807_B000 | 0x0807_F7FF | 18 KB | 9 | 应用可掉电存储（OTA 不动） |
| META | 0x0807_F800 | 0x0807_FFFF | 2 KB | 1 | 元数据 + 标志 |

- APP1 与 APP2 **等长**（各 230 KB = 115 页），因为 APP2 必须能装下整个 APP1 镜像。
- 当前 APP 镜像约 51 KB，230 KB 余量充足。
- **NV Storage（18 KB = 9 页）**：应用可掉电存储区，用于参数 / 日志 / EEPROM 仿真（可走 FlashDB / littlefs）。位于 APP1/APP2 之外，**OTA 不擦写，升级后数据保留**。
- 边界全部 2 KB 页对齐。

## 3. RAM 分配

- Bootloader 与 APP **不同时运行**，共用全部 128 KB SRAM，无需时间上分区。
- Bootloader 运行期占用：栈（1~2 KB）+ IAP 页缓冲（2 KB）+ UART/MB-RTU 收发缓冲（各 256 B）+ CRC 少量。
- CCM SRAM（32 KB @ 0x1000_0000）留给 APP 未来 FOC 高速数据，bootloader 不用。
- SRAM 布局：SRAM1 80 KB @ 0x2000_0000，SRAM2 16 KB @ 0x2001_4000，CCM 32 KB @ 0x1000_0000。

## 4. 链接地址与向量表

| 目标 | 链接地址 | `VECT_TAB_OFFSET` | VTOR |
| --- | --- | --- | --- |
| Bootloader | 0x0800_0000 | 0x0000 | 0x0800_0000 |
| APP | 0x0800_8000 | 0x8000 | 0x0800_8000 |

工程改动：

- 现有 APP 的 `bsp/linker/stm32g474xx_flash.sct`：`LR_IROM1 0x8000000 0x00080000` → `0x8008000 0x00039800`。
- 新建 bootloader 的 scatter：`LR_IROM1 0x8000000 0x00008000`。
- APP 的 `system_stm32g4xx.c`：定义 `USER_VECT_TAB_ADDRESS` + `VECT_TAB_OFFSET=0x8000`。
- 向量表地址是**编译期常量**，非运行时探测；bootloader 跳前 + APP `SystemInit` 双设置。

## 5. META 页结构

META 占最后一页 `0x0807_F800`（2 KB），用 struct 描述，其余空间保留：

```c
#define BOOT_META_ADDR         0x0807F800u
#define BOOT_META_MAGIC        0x4D435031u   /* "MCP1" */

/* flags 位定义 */
#define BOOT_FLAG_APP_VALID    (1u << 0)  /* APP 已通过校验 */
#define BOOT_FLAG_IAP_REQUEST  (1u << 1)  /* 请求进入 IAP */

typedef struct {
    uint32_t magic;         /* 固定 BOOT_META_MAGIC */
    uint32_t app_size;      /* APP 固件长度（字节） */
    uint32_t app_crc32;     /* APP 镜像 CRC32 */
    uint32_t flags;         /* 见上 */
    uint32_t reserved[12];  /* 保留，sizeof 凑整 64 字节 */
} boot_meta_t;
```

`sizeof(boot_meta_t) = 64` 字节，占页内偏移 0..63；`0x40..0x7FF` 保留。

### 5.1 标志写语义

- Flash 只能 1→0，要 0→1 必须先整页擦除。META 页是**专用页**，写标志 = 擦整页 + 重新编程 struct。
- 触发 META 写的事件：
  - COMMIT：擦写 META，置 `APP_VALID`，写 app_size/app_crc32，清 `IAP_REQUEST`；
  - APP 请求升级：写 `IAP_REQUEST`（APP 运行时或上位机触发）。

### 5.2 校验字段

Bootloader 校验只依赖 **magic + app_size + app_crc32**，不存版本号。

## 6. Flash 操作约束（供驱动实现）

- **擦除**：页擦除（2 KB），擦除后整页为 `0xFF`。
- **编程**：128-bit（16 字节）对齐、16 字节整数倍；不足 16 字节的末尾用 `0xFF` 补齐。
- 必须**先擦后写**；同一地址二次编程前必须重新擦除。
- bootloader 地址范围（0x0800_0000~0x0800_7FFF）**拒绝擦写**。
- NV Storage 区（0x0807_B000~0x0807_F7FF）由 APP 独占，bootloader 不擦写、不触碰。
