# ADR：自举电容预充电下沉到 mcl（方案 A：扩展 mcl_hal_ops）

| 项 | 值 |
|---|---|
| 文档名称 | mcl 架构决策记录：自举电容预充电下沉到 mcl |
| 编号 | ADR-mcl-001 |
| 状态 | 提议（Proposed）—— 仅定案设计，暂不实现 |
| 日期 | 2026-09-13 |
| 决定者 | 电机控制模块负责人 |
| 前置 | `docs/spec/mcl_architecture.md`（HAL 注入、库不碰硬件）、`docs/spec/mcl_spec.md`（FR / NFR） |

---

## 1. 背景与问题

无刷/永磁电机功率级采用**自举式（bootstrap）半桥驱动**（如 IR2101 / IR2102 / IR2110 分离式方案）时，高边 MOSFET 的栅极驱动供电依赖自举电容。自举电容仅在**下管导通**期间通过自举二极管充电；当上电首次启动、或电机长时间停转（高边长期导通、电容电荷漏光）后，高边驱动会欠压，导致上管无法正常开启或驱动不足。

当前工程现状：

- `hal/hal_tim1.c` 已提供 `hal_tim1_turn_on_low_sides(uint32_t ccr)`（注释注明「三相下管导通（自举电容充电，参考 ST R3_2_TurnOnLowSides）」），但**全工程无任何调用点**。
- 启动时序 `drv_motor_start()` 在 `hal_tim1_pwm_enable()` 直接开满 6 路 PWM，随后进入 R/L 实测与 `mcl_start()`，中间**没有自举预充电步骤**。

期望：自举预充电不应散落在 app / drivers 层手写，而应作为**功率级启动前置时序**由 mcl 统一承载，使换板 / 换驱动方案时该时序自动跟随。

---

## 2. 决策

采用**方案 A：扩展 `mcl_hal_ops`，新增可选回调 `precharge`**，由 mcl 在 `mcl_start()` 启动时序中调用。

> 否决方案 B（新增状态机阶段 `MCL_STATE_PRECHARGE`）：改动面大（状态机、查询、测试、文档全部波及），且自举充电是「一次性、阻塞式、启动前置」动作，语义上更适合由 HAL 回调承载，而非一个可观测的运行状态。

### 2.1 新增 HAL 回调

在 `include/mcl_hal.h` 的 `mcl_hal_ops` 中新增（可选）：

```c
/**
 * @brief 自举电容预充电（可选，NULL 表示无需 / 由门驱芯片自举自动完成）
 *
 * 让三相下管导通一段时间，为自举电容充电，保证高边驱动有足够电压。
 * 阻塞式实现；由 mcl 在 mcl_start() 启动时序中、正常 PWM 输出之前调用。
 * 非自举式驱动（电荷泵 / 集成自举的门驱 IC）可将此回调置 NULL，mcl 自动跳过。
 *
 * @param ctx         HAL 上下文
 * @param duration_us 建议充电持续时长（µs）；宿主可按实际电容/驱动调整
 */
void (*precharge_bootstrap)(void *ctx, uint32_t duration_us);
```

### 2.2 mcl 调用点

`mcl_start()` 在置 `state = RUN` 之前（即进入控制循环、开始输出正常 PWM 之前）调用：

```c
int mcl_start(mcl *self)
{
    /* ... 现有校验 ... */

    /* 自举预充电：可选回调，NULL 跳过；需在正常 PWM 输出前完成 */
    if (self->hal->precharge_bootstrap != NULL)
    {
        self->hal->precharge_bootstrap(self->hal_ctx, self->cfg.bootstrap_precharge_us);
    }

    self->state = MCL_STATE_RUN;
    /* ... */
}
```

### 2.3 新增配置字段

在 `mcl_config` 增加（可选，默认 0 表示用约定默认值）：

```c
uint32_t bootstrap_precharge_us;   /**< 自举预充电时长 µs（0=采用宿主默认）。 */
```

---

## 3. 宿主侧接线（drivers 层）

`drv_motor.c` 的 `s_mcl_hal` 中把新回调接到现有 `hal_tim1_turn_on_low_sides()`：

```c
static void drv_motor_precharge(void *ctx, uint32_t duration_us)
{
    (void)ctx;
    hal_tim1_turn_on_low_sides(PWM_HALF_PERIOD);   /* 下管全开 */
    /* 用 duration_us 做一个忙等待（复用 drv_motor_micros / wait_us） */
}
```

并把 `drv_motor_start()` 中「使能 MOE 之前先 precharge」的时序对齐到 mcl：宿主只需保证 `precharge_bootstrap` 回调内部自行完成「下管导通 + 按时长等待」，不再在 drivers 层手写充电时序。

---

## 4. 理由与权衡

| 维度 | 结论 |
|---|---|
| **符合「库不碰硬件」原则** | 回调注入，mcl 依旧芯片无关、不写寄存器，对齐 NFR-1 可移植性 |
| **换方案零成本** | 电荷泵 / 集成自举的门驱 IC 填 `NULL` 即跳过；分离式自举填回调实现 |
| **与现有 HAL 风格一致** | 复用 `void *ctx` 多实例铁律、返回约定，不新增抽象概念 |
| **时序正确性** | 预充电必须在正常 PWM 输出前、且一次性阻塞完成，`mcl_start()` 是唯一合适挂载点 |

待确认（实现前必须澄清）：

1. **实际驱动方案**：确认功率板确为 IR2101/2102/2110 分离式自举（当前仅从原理图 PDF 器件清单推断），否则 `precharge` 可能本就多余。
2. **`turn_on_low_sides` 的 ccr 语义**：需对照 PWM1 模式 + 极性确认「下管导通」是 CCR 取大值还是小值，避免写反导致上下桥直通风险。
3. **充电时长**：典型 1~10 ms，需按自举电容容值 + 驱动芯片充电电流核算，不要拍脑袋。

---

## 5. 影响范围

- `include/mcl_hal.h`：`mcl_hal_ops` 增加一个可选回调字段（结构体尾追加，向后兼容）。
- `include/mcl_config.h`：`mcl_config` 增加 `bootstrap_precharge_us` 字段。
- `src/mcl.c`：`mcl_start()` 增加一次可选调用。
- `mcl_types.h` / 裁剪宏：`precharge` 不依赖具体子模块，无新增裁剪宏。
- 测试：SIL mock HAL 增补一个 `precharge_bootstrap` 桩，验证「NULL 跳过 / 非 NULL 被调且时序先于 RUN」。
- 宿主 `drv_motor.c`：接线实现（本次不落地）。

---

## 6. 变更记录

| 版本 | 日期 | 说明 |
|---|---|---|
| v0.1.0 | 2026-09-13 | 初稿：定案方案 A（扩展 mcl_hal_ops 可选 precharge 回调），暂不实现 |
