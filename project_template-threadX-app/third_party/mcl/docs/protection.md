# MCL 保护模块

## 职责

`include/mcl_protection.h` 与 `src/mcl_protection.c` 是无 HAL 依赖的保护模块。
模块管理阈值检测、外部保护输入、首次故障锁存、现场快照、清除和恢复计时。
独立测试仅链接 `mcl_protection.c`，不需要 mcl.c、电机模型、观测器或板级驱动。

- MCL 控制层：采集电流/电压/温度/速度，提交 `mcl_protection_sample`，执行 FAULT/PWM 置零。
- BSP/HAL：Break、比较器等紧急关断，以及 DWT 中断超时检测；通过 `mcl_fault_assert()` 上报。
- AVS：仍独立负责运行中的制动电流限制，不作为故障锁存的一部分。
- 温度降额：继续通过 `mcl_protection_derate()` 返回系数，由控制层执行限流。

## 配置与接口

应用继续配置 `mcl_config.limits` 和 `mcl_config.fault_stop_time`。
`mcl_init()` 和 `mcl_set_config()` 将配置传入保护模块；运行时配置更新不会清除故障现场。

| 接口 | 作用 |
|---|---|
| `mcl_protection_init` | 初始化阈值并清空全部状态 |
| `mcl_protection_configure` | 更新阈值和自动清除时间，保留锁存和快照 |
| `mcl_protection_set_status` | 更新门驱、栅极电压、编码器/旋变等宿主输入 |
| `mcl_protection_update` | 阈值检测，锁存首次故障并记录采样快照 |
| `mcl_protection_assert` | 接收外部事件，不覆盖已经锁存的首次故障 |
| `mcl_protection_get_fault` / `get_fault_info` | 查询当前锁存 / 历史快照 |
| `mcl_protection_clear` | 显式确认故障，清锁存和计时，保留快照 |
| `mcl_protection_advance` | 推进物理秒恢复计时，返回是否已自动清除 |

旧 `mcl_protection_check()` 保留为无锁存的阈值检查接口。控制路径使用新的 `update()`。
门面 `mcl_get_fault()`、`mcl_get_fault_info()`、`mcl_fault_assert()`、`mcl_clear_fault()` 保持不变。
`motor.fault` 保留为只读兼容镜像，禁止直接赋值来清故障。
原 `motor.fault_info` / `motor.fault_timer` 已迁移为 `motor.protection.info` /
`motor.protection.recovery_elapsed_s`，推荐使用查询接口。结构体布局改变，需要全量重编译。

## 行为约定

保持原有保护使能位、故障码、阈值比较顺序与边界条件，不额外添加触发延时或滞回。
本次分离没有修改硬件关断方式。FOC 刷新温度与零偏时保留宿主设置的其他保护状态；
宿主需要显式更新或清除这些输入，不再每拍被清零覆盖。

软件检测与硬件上报共用一个首次故障快照。重复上报仍执行停机，但不会改写快照或重置恢复计时。
清除后保留历史快照，下次新故障覆盖它；重新初始化才清空历史。

`fault_stop_time=0` 禁止自动清除（当前板级配置如此）。大于零保留原有延时确认行为：
到时清锁存并进入 IDLE，**不自动启动电机**。手动 `mcl_clear_fault()` 在 RUN 状态返回错误。
清除操作不代表硬件故障已消失；持续的故障输入会在再次检测时重新触发。
本轮没有新增“所有条件恢复后才允许确认”的策略，避免分离模块时改变已有恢复语义。
时间始终使用 float 物理秒，与 Q15/Q31 的算法 dt 标幺值分离。

这些接口不实现跨线程/嵌套中断锁。配置、清除和外部状态更新应由宿主与控制中断串行化；
硬件紧急关断必须先在硬件/HAL 层完成，不依赖软件模块的执行时机。

## 验证

- float/Q15/Q31 独立模块测试：阈值边界、锁存、首次现场、配置更新、清除、物理秒计时、持续故障再次触发。
- 集成故障测试：PWM 置零、重复上报、故障禁止启动、自动回 IDLE、手动清除、外部输入保留。
- 现有无感启动、AVS、保护及三精度板级模型回归通过。
- 当前 float + O0 整板全量编译：0 errors / 0 warnings。未烧录验证。
