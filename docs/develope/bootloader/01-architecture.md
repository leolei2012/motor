# 01 — Bootloader 架构与决策

## 1. 目标

- 通过 Modbus RTU（复用现有 USART2）实现固件在线升级（IAP / OTA）。
- 升级失败（校验失败 / 中途断电）时设备可恢复，**永不真正变砖**。
- 只维护**一份固件镜像**、**一个链接地址**，不增加上位机复杂度。

## 2. 非目标（首版不做）

- 双 bank 硬件无感切换 / 后台升级（升级时设备停机，电机先安全停）。
- 自动回滚（新固件运行异常自动切回旧版）。
- 运行确认（run-confirm）机制。
- 加密 / 签名（只有 CRC32 完整性校验）。

## 3. 架构总览

```text
                    ┌──────────────────────────────────────────────┐
                    │                Flash (512 KB)                 │
                    │                                               │
0x0800_0000  Bootloader  32 KB   ── 固定入口，永不自擦，永远活着      │
0x0800_8000  APP1        230 KB  ── 运行槽（固件唯一链接地址）          │
0x0804_1800  APP2        230 KB  ── 下载缓冲（只存字节，不执行）        │
0x0807_B000  NV Storage  18 KB   ── 应用可掉电存储（参数/日志，OTA 不动）│
0x0807_F800  META        2 KB    ── 元数据 + 标志                     │
                    └──────────────────────────────────────────────┘
```

```text
上电 → BOOT_CHECK
         ├─ APP1 有效 且 无 IAP 请求 ──> 跳 APP1
         └─ 强制 IAP / APP1 无效 ─────> IAP 从机
                                          ├─ 收固件 → 写 APP2（攒 2KB 页）
                                          ├─ CRC32(APP2) 校验
                                          ├─ copy APP2 → APP1
                                          ├─ 写 META（APP_VALID）
                                          └─ 复位 → 跳 APP1
```

## 4. 关键决策（ADR）

### ADR-1：单 bank 软件双槽 + copy，不用硬件双 bank

- **决策**：保持单 bank（`DBANK=0` 默认），用两个软件槽 APP1/APP2，下载后 copy APP2→APP1。
- **理由**：
  - STM32G474 双 bank 只有 `BFB2`（启动时选 bank2），**没有运行时 swap bank**；
  - `BFB2` 要擦写 option byte + 复位，且 bootloader 得在两个 bank 各放一份，复杂度高；
  - 页擦除粒度 2 KB，擦 APP2 的页根本碰不到 bootloader / APP1，隔离靠页擦除即可。
- **被否方案**：硬件双 bank（BFB2）、bank 交换。

### ADR-2：固件唯一链接地址 = APP1

- **决策**：固件只在 `0x08008000` 链接，APP2 只当字节缓冲。
- **理由**：一个 .bin 只能跑在它链接的地址；若跑在两个地址，要么两份镜像、要么 PIC，都不可取。
- **被否方案**：两份链接镜像；位置无关代码（Cortex-M + Keil 不实际）。

### ADR-3：下载到 APP2 → 校验 → copy 到 APP1

- **决策**：新固件先写入 APP2，CRC 通过后再 copy 到 APP1。
- **理由**：下载阶段 APP1 原封不动，下载断电不影响旧固件；风险只剩 copy 那几秒。
- **补充**：copy 中若断电，APP1 半成品 → 下次上电 APP 无效 → 进 IAP 重新下载（**不做重 copy**，越简单越好）。
- **被否方案**：
  - 流式直写 APP1：下载断电即毁 APP1；
  - 收进 RAM 再写：受 128 KB RAM 上限约束，而 APP2(Flash 230KB) 更大且掉电不丢。

### ADR-4：向量表 = 编译期常量，VTOR 双设置

- **决策**：向量表地址就是链接地址，用 `VECT_TAB_OFFSET` 编译期常量；bootloader 跳前设一次、
  APP 的 `SystemInit()` 再设一次（权威值）。
- **理由**：VTOR 不是运行时探测，而是「告诉 CPU 中断入口在哪」，永远等于链接地址。
- **澄清**：Cortex-M4 支持 `SCB->VTOR` 重定位；「STM32 不能改向量表」是误解。

### ADR-5：复用 USART2 + MB-RTU FC41 + uart_control

- **决策**：IAP 通道 = 现有 USART2（9600）+ 已集成的 MB-RTU FC41 IAP 从机 + uart_control 零拷贝收发。
- **理由**：协议和驱动都已就绪，bootloader 只需实现 `on_start/on_data/on_end` 三个回调。

### ADR-6：调试接口 = probe_station 库（设备侧 Modbus 调试接口）

- **决策**：设备侧的调试接口收敛到 `third_party/probe_station` 库，专门通过 Modbus RTU 从站暴露调试寄存器（状态 / 参数 / 分区 / 进度），供 ProbeStation 平台 / AI agent 调试。
- **依赖链**：`probe_station → modbus(MB-RTU) → uart_control`。
  - probe_station 只依赖 modbus（`base.send` 回调），不直接碰串口驱动；
  - modbus 通过发送回调与 uart_control 解耦；
  - uart_control 由 drv_usart2 封装提供收发。
- **适配器模式**：各固件（bootloader / motor_control）提供自己的适配器（寄存器映射 + init/poll 钩子），如 bootloader 的 `ps_adapter_bootloader`（状态段 0x0000 + 调试段 0xE000 + FC41 IAP）。

## 5. Bootloader 功能清单

1. 上电最小初始化：时钟、GPIO（LED/按键）、USART2、TIM7（帧超时）；**不启动**电机/TIM1/ADC。
2. 上电判定（BOOT_CHECK）：**先验 META 的 magic，再读 flags**（见 §5.1）。
3. Modbus FC41 IAP 从机（详见 03-iap-protocol）。
4. Flash 驱动：2 KB 页擦除 + 128-bit 编程 + 读；**保护 bootloader 自己的 16 页不被擦写**。
5. CRC32：标准 CRC-32（poly `0xEDB88320`，初值/终值 `0xFFFFFFFF`，大端）。
6. 跳转 APP1（见 §7）。

### 5.1 上电判定（BOOT_CHECK）与 BOOT_REASON

全新芯片 / 空 META 页是全 `0xFF`：`magic=0xFFFFFFFF`、`flags=0xFFFFFFFF`（所有标志位看起来都置位）。
因此必须**先验 magic，再读 flags**，否则空 META 会被误判成「IAP_REQUEST 已置位」。

```c
typedef enum {
    BL_REASON_APP_VALID   = 0,  /* APP 有效，正常跳转 */
    BL_REASON_APP_INVALID = 1,  /* APP CRC/size 校验失败 */
    BL_REASON_IAP_REQUEST = 2,  /* 用户强制 IAP */
    BL_REASON_NO_APP      = 3,  /* 空 META（首次生产）/ META 损坏 */
} bl_boot_reason_t;
```

```c
void bl_boot_check(void)
{
    boot_meta_t meta;
    bl_meta_read(&meta);

    bool meta_valid = (meta.magic == BL_META_MAGIC);           /* 空 META → false */
    bool app_valid  = meta_valid && bl_meta_app_valid(&meta);  /* 空 META → false */
    bool iap_req    = meta_valid && (meta.flags & BL_FLAG_IAP_REQUEST);

    if (app_valid && !iap_req) {
        bl_jump_to_app();                          /* 正常跳转 */
    } else {
        bl_boot_reason_t reason =
            !meta_valid ? BL_REASON_NO_APP        /* 首次 / META 损坏 */
            : iap_req  ? BL_REASON_IAP_REQUEST    /* 用户强制 */
                       : BL_REASON_APP_INVALID;   /* APP 校验失败 */
        bl_enter_iap(reason);                     /* 进 IAP，reason 上报到状态寄存器 */
    }
}
```

- **首次生产**：空 META → `meta_valid=false` → 自动进 IAP，reason=`NO_APP`，无需任何特殊操作。
- commit 之后 META 被写好（magic/size/crc/APP_VALID），META 不再为空。

## 6. 状态机

```text
                    ┌─────────────┐
                    │    RESET    │
                    └──────┬──────┘
                           v
                   ┌───────────────┐
                   │  BOOT_CHECK   │  先验 magic → APP 有效？IAP 请求？
                   └───┬───────┬───┘
            IAP/无效  │       │ APP1 有效
                      v       v
             ┌────────────┐  ┌─────────────┐
             │  IAP_IDLE  │  │ JUMP_TO_APP │
             │  等 START  │  │ VTOR=APP1   │
             └─────┬──────┘  └─────────────┘
                   │ START
                   v
             ┌────────────┐
             │ RECEIVING  │  on_data：攒 2KB 页写 APP2
             └─────┬──────┘
                   │ END
                   v
             ┌────────────┐
             │   VERIFY   │  CRC32(APP2) == expected ?
             └─────┬──────┘
               fail│    │ pass
             (回 IAP_IDLE) v
             ┌────────────┐
             │    COPY    │  APP2 → APP1（页擦 + 编程）
             └─────┬──────┘
                   v
             ┌────────────┐
             │   COMMIT   │  写 META：APP_VALID / size / crc，清 IAP_REQUEST
             └─────┬──────┘
                   v
             ┌────────────┐
             │   RESET    │  （或直接 JUMP）
             └────────────┘
```

## 7. 跳转 APP（JUMP_TO_APP）

```c
void boot_jump_to_app(uint32_t app_base)
{
    uint32_t sp = *(volatile uint32_t *)app_base;        /* APP 初始 SP      */
    uint32_t pc = *(volatile uint32_t *)(app_base + 4);  /* Reset_Handler    */

    __disable_irq();                 /* 1. 关全局中断          */
    /* 2. 复位用到的外设（USART2/TIM7/DMA），可选但推荐 */
    SCB->VTOR = app_base;            /* 3. 向量表重定位         */
    __set_MSP(sp);                   /* 4. 设主栈指针           */
    ((void (*)(void))pc)();          /* 5. 跳 Reset_Handler     */
}
```

APP 侧在 `system_stm32g4xx.c` 的 `SystemInit()` 中：`SCB->VTOR = FLASH_BASE | 0x8000`。

## 8. 风险与兜底

| 风险 | 兜底 |
| --- | --- |
| 下载中断电 | APP1 原封不动，下次上电照常跑旧固件 |
| copy 中断电 | APP1 半成品（CRC 不过）→ 停在 IAP 等重下 |
| 校验失败 | 停在 IAP 等重传，APP1 不受影响 |
| bootloader 被误擦 | flash 驱动拒绝擦写 bootloader 地址范围 |
| 上位机发错固件 | CRC32 不匹配 → 拒绝 commit |
