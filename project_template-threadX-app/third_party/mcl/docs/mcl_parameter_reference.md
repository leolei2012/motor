# MCL 模块变量与参数汇总

核对日期：2026-09-29。对应当前工作区源码（含宿主 setter/getter 接口），不是历史发布版。本文中的“板级值”指 `drv_motor_config_default()` 填入的初始值，不代表运行时读取值；校准、setter 和监控指令可能改变当前值。

源码入口：[集中配置](../include/mcl_config.h)、[库默认值与校验](../src/mcl_config.c)、[门面 API](../include/mcl.h)、[板级配置](../../../drivers/motor/include/drv_motor_config.h)、[板级基值](../../../drivers/motor/include/drv_motor_units.h)。访问规则见 [host_api.md](host_api.md)。

## 1. 阅读与访问约定

参数是可配置输入；状态是控制算法每拍维护的结果。`mcl` 和子模块结构体公开是为了静态分配，宿主不能据此直接修改内部字段。初始化前可以填写宿主持有的配置对象；初始化后用 getter 读取副本、setter 提交修改，并检查返回值。

以下修改方式缩写贯穿配置表：

| 标记 | 读取方式 | 修改方式与时机 |
|---|---|---|
| C | `mcl_get_config()` | `mcl_set_config()`；允许运行时更新，但必须与控制中断串行化 |
| I | `mcl_get_config()` | 初始化时填写；停机后重新 `mcl_init()`，`mcl_set_config()` 不采纳该字段 |
| M | `mcl_get_motor_parameters()` | `mcl_set_motor_parameters()`；仅 IDLE，当前支持 SMO 或无观测器，同步配置、FOC/MTPA 和 SMO |
| O | `mcl_get_current_offsets()` | `mcl_calibrate_offset()`；停机校准；或初始化时填写 |
| S | `mcl_observer_smo_get_params()` | `mcl_observer_smo_set_params()`；观测器停用时，与 update 串行化 |

库 getter 不自动屏蔽中断。读取多字段快照或更新参数时由宿主保证一致性；本工程的 `drv_motor_get_diagnostics()` 封装了电机与 SMO 快照的临界区。不要把 getter 返回的副本修改误认为已经修改了电机。

## 2. 类型、单位和基值

未定义精度宏时 `mcl_scalar=float`；`MCL_USE_Q15` 为 `int16_t` Q1.15；`MCL_USE_Q31` 为 `int32_t` Q1.31。定点有效范围为 `[-1,1)`，表中的 `1*` 表示最大正定点值，略小于 1。Q15/Q31 默认值列写的是解码后的标幺目标值，实际会量化。

| 量 | float 单位 | 定点除数：当前板级基值 |
|---|---|---|
| 电压 | V | `V_BASE=128 V` |
| 电流 | A | `I_BASE=32 A` |
| 电气角速度 | rad/s | `W_BASE=2048 rad/s` |
| 机械转速 | rpm | `RPM_BASE=W_BASE×60/(2π×5)`，约 3911.39 rpm |
| 角度 | rad | `ANGLE_BASE=2π`，定点值表示圈数 |
| 电阻 | Ω | `R_BASE=V_BASE/I_BASE=4 Ω` |
| 电感 | H | `L_BASE=R_BASE/W_BASE=0.001953125 H` |
| 磁链 | Wb | `FLUX_BASE=V_BASE/W_BASE=0.0625 Wb` |
| 算法积分周期 `dt` | s | `TIME_BASE=1/W_BASE=0.00048828125 s` |
| 温度 | ℃ | `TEMP_BASE=128 ℃` |
| 转矩指令 | N·m | `FLUX_BASE×I_BASE`；极对数系数由转矩换算处理 |

**时间有两类例外：** `openloop_hyst/time_lock/time_ramp/time`、`fault_stop_time` 及运行时状态机计时器是 `float` 物理秒，三精度都不归一化。`avs_recovery_time`、`speed_aw_time` 虽然是 `mcl_scalar`，也保持物理秒，定点编码基值为 1，不能表达大于等于 1 秒的完整正值。

常量使用 `MCL_CONFIG_VALUE(物理值, 基值)`；动态值使用 `mcl_from_physical()` / `mcl_to_physical()`。`MCL_FROM_FLOAT()` 只编码数值，不会自动选择物理基值。不要直接把 24 V 或 1000 rpm 转成 Q31。整型计数、布尔值和枚举不做标幺转换。

## 3. 集中配置 `mcl_config`

除特别注明，字段类型均为 `mcl_scalar`。“同”表示同本行 float 默认值；“继承”表示板级函数未覆盖，定点下仍继承定点模板，不能按 float 物理值理解。

### 3.1 电机与调度

| 字段 | 意义/单位 | 库 float 默认 | 库 Q15/Q31 默认 | 当前板级物理值 | 访问 |
|---|---|---:|---:|---:|---|
| `pole_pairs` | 极对数，uint8_t | 4 | 同 | 5 | I |
| `phase_resistance` | 相电阻，Ω | 1 | 0.5 | 0.475 | M |
| `phase_inductance` | Lq，H | 0.001 | 0.5 | 0.0008 | M |
| `ld_lq_diff` | Lq−Ld，H | 0 | 0 | 0.000075 | M |
| `bemf_const` | 永磁磁链，Wb | 0.02 | 0.5 | 0.00717 | M |
| `rated_current` | 额定峰值电流，A | 5 | 0.9 | 4 | I |
| `rated_speed_rpm` | 额定机械转速参考，rpm | 3000 | 0.9 | 3000 | I |
| `pwm_freq_hz` | PWM 频率，uint32_t Hz | 20000 | 同 | 16000 | I |
| `current_loop_freq_hz` | tick 频率，uint32_t Hz | 20000 | 同 | 16000 | I |
| `speed_loop_divider` | 速度环分频，uint8_t | 10 | 同 | 16（1 kHz） | I |
| `pos_loop_divider` | 位置环分频，uint8_t | 10 | 同 | 继承（1.6 kHz） | I |
| `max_duty` | 调制输出限制，无量纲 | 0.95 | 同 | 继承 | C |
| `bus_voltage` | 标称母线电压，V，非实测值 | 24 | 1* | 24 | I |
| `time_base` | 积分时间基值，s | 1 | 1* | float=1；定点=1/2048 | I |
| `current_offset[3]` | abc 采样零漂，A | 全 0 | 全 0 | 校准前全 0 | O |

库定点默认电机参数是归一化模板，不是同一台默认电机的物理参数。硬件 PWM/ADC 时序仍由宿主配置，改 `pwm_freq_hz` 不会自动改定时器。

### 3.2 PID 与指令斜坡

三个 `mcl_pid_params` 都含 `kp`、`ki`、`kd`、`out_min`、`out_max`、`i_min`、`i_max`。均按 C 方式访问；`out_*` 为输出限制，`i_*` 为积分项限制。

| 配置 | 库 float：kp / ki / kd | 库定点：kp / ki / kd | 库 float 输出/积分限幅 | 库定点限幅 | 当前板级 float |
|---|---|---|---|---|---|
| `current_pid` | 1 / 100 / 0 | 1* / 0.05 / 0 | ±1 | −1..1* | 0.04 / 4 / 0；限幅 ±1 |
| `speed_pid` | 0.5 / 20 / 0 | 0.5 / 0.01 / 0 | ±2 A | ±0.9 | 0.005 / 0.02 / 0；限幅 ±3 A |
| `pos_pid` | 100 / 20 / 0 | 0.5 / 0.01 / 0 | ±500 rpm | ±0.9 | 继承，非本板已整定参数 |

速度环误差为机械 rpm，输出为 A；位置环误差为角度，输出为 rpm。当前 float 电流 PI 输出为调制电压量，增益由 0.48 V/A 和 48 V/(A·s) 除以 12 V 得到。定点电流 PI 先输出 `V/V_BASE`，再换算调制量，不能对 float 增益简单使用基值 1。

| 当前板级增益 | 定点编码前的换算 |
|---|---|
| 电流 `kp` | `0.04 / (V_BASE/(12×I_BASE)) = 0.12` |
| 电流 `ki` | `4 / (V_BASE×W_BASE/(12×I_BASE)) = 0.005859375` |
| 速度 `kp` | `0.005 / (I_BASE/RPM_BASE)` |
| 速度 `ki` | `0.02 / (I_BASE×W_BASE/RPM_BASE)` |

`cfg` 保存配置增益；定点外环内部还会把 Ki 按外环周期离散化，所以 `pid_speed.params.ki` 不应拿来替代 `mcl_get_config()` 的结果。

| 字段 | 意义 | 库 float / 定点默认 | 板级值 | 访问 |
|---|---|---|---|---|
| `speed_ramp_rpm_s` | 目标转速变化率；0 不斜坡 | 0 / 0 | 500 rpm/s；定点除 RPM_BASE | C |
| `speed_aw_time` | 速度 PI 回算时间；0 关闭该回算机制 | 0 / 0 | 0.02 s，编码基值 1 | C |

### 3.3 AVS 限回馈

| 字段 | 意义 | 库 float 默认 | 库定点默认 | 板级值 | 访问 |
|---|---|---|---|---|---|
| `avs_enabled` | bool 开关 | false | false | **true** | C |
| `avs_start_voltage` | 开始减小制动电流 | 27 V | 0.85 | 28 V | C |
| `avs_stop_voltage` | 回馈电流缩减至零的电压 | 29 V | 0.90 | 30 V | C |
| `avs_recovery_time` | 恢复制动力时间 | 0.05 s | 0.05 s | 0.05 s | C |
| `avs_speed_deadband` | 回馈方向判断速度死区 | 5 电气 rad/s | 0.01 | 5 电气 rad/s | C |

启用时校验：`bus_voltage < start < stop < limits.overvoltage`，恢复时间和 `speed_aw_time` 必须大于 0，死区非负，速度 PI 输出范围包含零，硬过压保护必须开启。AVS 限制与旋转方向相反的 Iq，不等于母线电压硬钳位。关闭 AVS 不会自动关闭独立的速度环抗积分饱和和指令斜坡。

### 3.4 反馈与 PLL

| 字段 | 意义 | 库 float 默认 | 库定点默认 | 板级值 | 访问 |
|---|---|---|---|---|---|
| `feedback.type` | `mcl_feedback_type` | NONE | 同 | NONE（无感） | I |
| `feedback.encoder_offset` | 电角度偏置 | 0 rad | 0 圈 | 继承 | I；也可停机对齐校准 |
| `feedback.encoder_cpr` | uint32_t 编码器计数配置 | 4096 | 同 | 继承，无感不用 | I |
| `pll_kp` | 相位误差比例反馈 | 2000 | 0.3 | 200；定点除 `W_BASE/ANGLE_BASE` | I |
| `pll_ki` | 相位误差积分反馈 | 30000 | 0.01 | 10000；定点除 `W_BASE²/ANGLE_BASE` | I |

特别注意：当前 `mcl_set_config()` **不更新 PLL 增益**。

### 3.5 自动开环启动

| 字段 | 意义 | 库 float 默认 | 库定点默认 | 板级值 | 访问 |
|---|---|---|---|---|---|
| `openloop_rpm` | 拖动目标机械转速 | 200 rpm | 0.3 | 300 rpm | C |
| `openloop_min_rpm` | 启动机械转速下限 | 15 rpm | 0.005 | 15 rpm | I |
| `openloop_rpm_low` | 预留自适应转速比例 | 0 | 0 | 继承，当前未启用 | I |
| `openloop_hyst` | 低速迟滞时间，float 秒 | 0.1 | 同 | 继承 | I |
| `openloop_time_lock` | 锁定时间，float 秒 | 0.05 | 同 | 0 | I |
| `openloop_time_ramp` | 加速时间，float 秒 | 0.1 | 同 | 1.5 | I |
| `openloop_time` | 匀速保持时间，float 秒 | 0.05 | 同 | 0.05 | I |
| `openloop_boost_q` | 预留电流增强项 | 0 | 0 | 继承，当前未启用 | I |
| `openloop_max_q` | 预留自适应电流上限 | −1 | −1 | 继承，当前未启用 | I |
| `openloop_drag_q` | 固定 I/F 拖动 Iq | 1 A | 0.5 | 3 A | C |
| `openloop_seed_angle` | 交接初始化角偏置 | π/4 rad | 0.125 圈 | 约 π/2；当前 SMO 路径不用此固定角 | C |

以实现为准：库 seed 默认是 **45°**，不是旧注释中的 90°；当前 SMO 交接取观测器已收敛的角度。上述计时参数均为物理秒。

## 4. 保护模块

### 4.1 `cfg.limits` 与恢复策略

阈值均为 `mcl_scalar`，使用 C 方式更新，读取 `mcl_get_config()`。使能 `enabled` 为 uint32_t 位掩码；库默认 `MCL_PROTECT_ALL` 实际只包含基础过流、过压、欠压、过温四项，不是全部扩展保护。

| `limits` 字段 | 单位 | 库 float 默认 | 库定点默认 | 板级值 | 板级启用 |
|---|---|---:|---:|---|---|
| `overcurrent` | A | 10 | 0.9 | 4 | 是 |
| `overvoltage` | V | 30 | 1* | 60 | 是 |
| `undervoltage` | V | 8 | 0.5 | 8 | 是 |
| `temp_derate_start` | ℃ | 80 | 0.5 | 80 | 温度保护未启用 |
| `overtemp` | ℃ | 100 | 0.8 | 80 | 否 |
| `abs_overcurrent` | A | 20 | 0.95 | 8 | 是 |
| `overtemp_fet` | ℃ | 100 | 0.8 | 继承 | 否 |
| `overtemp_motor` | ℃ | 100 | 0.8 | 继承 | 否 |
| `gate_overvoltage` | V | 20 | 0.95 | 继承 | 否 |
| `gate_undervoltage` | V | 8 | 0.2 | 继承 | 否 |
| `sincos_min` | 宿主约定幅度单位 | 0.2 | 0.05 | 继承 | 否 |
| `sincos_max` | 同上 | 1.5 | 0.95 | 继承 | 否 |
| `offset_max` | A | 2 | 0.2 | 2 | 否 |
| `unbalanced_max` | A | 3 | 0.2 | 3 | 是 |
| `overspeed` | 电气 rad/s | 800 | 0.8 | 1047，约 2000 机械 rpm | 是 |
| `underspeed` | 电气 rad/s | 0 | 0 | 继承 | 否 |
| `abs_overspeed` | 电气 rad/s | 1100 | 0.95 | 1100，约 2101 机械 rpm | 是 |

板级还未启用 DRV、BRK、旋变异常输入的周期检测。`cfg.fault_stop_time` 为 float 秒，库默认 1，板级 0（锁存，手动清除），使用 C 方式更新。自动清除只回 IDLE，不会自动启动，也不是确认外部故障已经消失。60 V 是当前软件配置值，不代表硬件额定耐压。

### 4.2 保护输入与故障快照

| 对象/字段 | 意义与单位 | 宿主接口 |
|---|---|---|
| `mcl_protection_status.temp_fet/temp_motor` | 温度，℃/温度标幺 | `mcl_set/get_protection_status()`；控制路径可由 HAL 温度刷新 |
| `gate_voltage` | 门驱电源电压，V/电压标幺 | 同上 |
| `sincos_amplitude` | 正余弦幅度，与阈值同尺度 | 同上 |
| `current_offset[3]` | 零漂，A/电流标幺 | 同上；控制路径会从配置刷新 |
| `drv_fault/brake_fault/resolver_lot/resolver_dos/resolver_los` | uint8_t 异常标志 | 同上；未更新的外部状态会保留 |
| `mcl_fault_info.fault` | 首次锁存的故障码 | `mcl_get_fault_info()` |
| `current/voltage/speed/temp/tick` | 故障现场：FOC 的 Iq、母线、电气速度、温度、tick 计数 | 同上；tick 不是毫秒 |
| 当前故障 | 清除后为 NONE；历史快照仍可保留 | `mcl_get_fault()`、`mcl_fault_assert()`、`mcl_clear_fault()` |

故障码常用值：1 过流、2 过压、3 欠压、4 过温、7 绝对过流、17 电流不平衡、**22 超速**、23 欠速、24 绝对超速。完整枚举见附录和 [protection.md](protection.md)。

## 5. SMO、PLL、FOC 与其他算法

### 5.1 SMO 参数

SMO 没有独立的一套通用默认参数，由调用方传入。当前板级为：

| `mcl_observer_smo_params` 字段 | 意义/单位 | 板级物理值 | 访问 |
|---|---|---:|---|
| `resistance` | 相电阻，Ω | 0.475 | 电机同步修改用 M |
| `inductance` | Lq，H | 0.0008 | M |
| `ld` | Ld，H；0 时按 Lq | 0.000725 | M，由 Lq−差值同步 |
| `flux` | 永磁磁链，Wb | 0.00717 | M |
| `gain` | 滑模电压上限，V | 10 | S |
| `lpf` | 滤波自适应速度下限，电气 rad/s | 6.28 | S |
| `boundary` | 线性滑模区电流误差，A | 0.5 | S |

S 接口校验有限数值，R≥0、Lq>0、0≤Ld≤Lq、flux>0、gain>0、boundary≥0、lpf≥0。修改实体参数优先使用 M，避免仅修改 SMO 导致 FOC 与观测器模型不同。

| SMO 状态 | 意义/单位 | 查询 |
|---|---|---|
| `i_alpha_hat/i_beta_hat` | 估计电流，A/I_BASE | SMO diagnostics |
| `e_alpha/e_beta` | 一级滤波 EMF，V/V_BASE | 内部 |
| `e_alpha_final/e_beta_final` | 二级滤波 EMF，V/V_BASE | SMO diagnostics |
| `phase` | 补偿后的电角度，rad/圈 | SMO diagnostics |
| `w_est` | 平滑原始角增量，**rad/tick，不是 rad/s** | SMO diagnostics |
| `dtheta_prev` | 前一拍原始角增量，rad/tick | 内部 |
| `theta_prev` | 前一拍未补偿 EMF 角，rad/圈 | 内部 |
| `z_alpha/z_beta` | 滑模修正电压，V/V_BASE | diagnostics 仅提供 z_alpha |
| `seed_omega` | 启动速度提示，电气 rad/s/W_BASE；0 释放 | 库交接逻辑维护 |
| `filter_step` | 低通系数，0..0.5 | 内部 |
| `dt` | 控制周期，秒/归一化时间 | 内部 |
| `i_alpha_last/i_beta_last` | 前拍实测电流，A/I_BASE | 内部 |
| `filter_remainder[6]` | 仅 Q15：滤波量化余数，int32_t | 内部 |

“SMO diagnostics” 指 `mcl_observer_smo_get_diagnostics()`。二级 EMF 的幅度受观测器反馈和滤波影响，不能直接当作未经滤波的真实反电动势幅值。

### 5.2 子模块状态与参数来源

| 模块 | 参数/状态 | 来源、单位及访问边界 |
|---|---|---|
| PID | `params`；`i_term/prev_error/prev_out`；定点 `aw_remainder/aw_tracking_remainder` | 参数来自 cfg 三组 PID；状态分别为输出单位、误差单位、输出单位和内部余数；无宿主积分项 setter |
| FOC | `pid_d/pid_q/mtpa_fw/phase_resistance/phase_inductance` | 从 cfg 初始化；参数通过 C/M 同步；不要直接改嵌套实例 |
| MTPA/弱磁 | `ld/lq/lambda/i_max/fw_id_min` | H、H、Wb、A、A（定点按对应基值）；初始化 `fw_id_min=-i_max`，板级初始 −4 A；无独立宿主限值 setter |
| PLL | `kp/ki/phase/speed/last_phase/speed_est_fast` | 增益取 cfg；角为 rad/圈、速度为电气 rad/s/标幺；Q15 另有积分余数；`pll_last_phase` 可从电机 diagnostics 读 |
| Flux | 参数 `lambda/resistance/inductance/gain/ld`；状态 `x1/x2/lambda_est/i_alpha_last/i_beta_last` | 磁链、电阻、电感按对应基值；gain 为磁链幅值校正增益，0 关闭校正；当前板未使用，无成对宿主参数 getter/setter |
| Ortega | 同类电机参数；磁链状态、电流历史和 `r_est/r_est_state/speed` | 当前板未使用；通过其初始化接口传参数，没有完整宿主参数访问对 |
| BLDC | `step/hall_map[8]/invert/bemf_integrator/bemf_threshold` | 换相步、霍尔映射、方向、反电动势积分及阈值；当前板 FOC 路径不用，无完整宿主参数访问对 |
| 通用 observer | `ops/impl/params` | 算法操作表、实例指针、初始化参数指针；由 mcl_init 注入，不是监控数值 |
| AVS helper | 无独立公开实例 | 状态在 mcl 的 avs_* 字段；配置见 3.3 |
| 变换/数学/SVPWM | 无公开持久参数结构体 | 输入输出为坐标电流、电压、角度或调制量；不应当作可写参数寄存器 |
| 校准 | 无独立持久实例 | 停机调用 mcl_calibrate_*；测量结果和采样偏置通过输出参数/配置 getter 读取 |
| HAL | `mcl_hal_ops` 回调表 | PWM、相电流、母线、编码器、霍尔、相电压、微秒计时、温度；初始化注入 |

这些旧模块“没有成对接口”是当前接口覆盖边界，不代表建议宿主直接操作它们的实例字段。

## 6. 电机运行状态、指令与诊断

**接口更新：** 以下电机运行状态的全部 37 个字段均已有独立 `mcl_get_<字段名>()`，直接返回原类型；例如 `mcl_get_avs_scale()`、`mcl_get_iq_applied()`、`mcl_get_ol_timer()`。下表描述原有批量接口的覆盖情况；标为内部的运行字段现在也能通过单值 getter 读取。完整列表及并发约定见 [host_api.md](host_api.md#逐变量运行状态-getter)。

| 状态字段/数据组 | 意义 | 访问方式 |
|---|---|---|
| `state/fault` | IDLE/ALIGN/RUN/FAULT；当前故障 | `mcl_get_state/get_fault()` 或 diagnostics |
| `mode/ctrl_mode` | 有感/无感/BLDC；电流/速度/位置/VF/IF/ALIGN 控制 | diagnostics；`mcl_set_mode()` 和各指令 setter |
| `iq_ref` | 当前 q 轴参考，A/标幺；未必等于最终限流后的值 | diagnostics；`mcl_set_current()` 或其他控制环生成 |
| `speed_ref_rpm/pos_ref_rad` | 最终速度目标、位置目标 | diagnostics；`mcl_set_speed/set_position()` |
| `speed_ramp_rpm` | 斜坡后的速度目标 | 单值 getter |
| `phase_rad/speed_rad_s` | 控制使用的电角度、电气角速度 | diagnostics/telemetry |
| `fb_speed_filt` | 速度环滤波反馈 | 内部 |
| `id_now/iq_now` | dq 实测电流，A/标幺 | diagnostics/telemetry |
| `id_cmd` | MTPA d 轴指令斜坡 | 内部 |
| `vbus/duty_now` | 实测母线、调制输出量 | diagnostics/telemetry |
| `v_alpha_prev/v_beta_prev` | 上拍由最终 duty 重建的电压调制量，按 Vbus/2 归一化 | diagnostics；不是 V/V_BASE |
| `avs_scale` | 当前允许回馈比例 0..1 | 单值 getter |
| `avs_recovery_step/speed_aw_gain` | 每电流拍恢复量、每速度拍回算比例 | 内部，由配置计算 |
| `iq_applied` | AVS/温度限制后实际应用 Iq | 单值 getter |
| `openloop_speed/openloop_angle/openloop_mag` | 开环速度、相位、幅值；VF 幅值为调制量，IF/ALIGN 为电流 | diagnostics；`mcl_set_openloop_vf/if/align()` |
| `openloop_phase` | 固定对齐相位 | 内部，由 align 指令设置 |
| `ol_speed/ol_phase` | 自动启动的速度与相位 | 内部 |
| `ol_stage/ol_started_once` | 启动阶段、已启动标记 | diagnostics 提供 ol_stage；其他内部 |
| `ol_timer/ol_hyst_timer/ol_anchor_timer/ol_lock_timer/ol_wait_timer` | 启动、迟滞、锚定和收敛计时 | 内部，float 物理秒 |
| `switch_blend_timer/switch_blend_iq0/switch_phase_offset` | 闭环交接时间、电流起点、角度偏差 | 内部；秒、电流、角度 |
| `tick_count/dt` | tick 计数、积分步长 | diagnostics 提供计数；`mcl_get_control_frequency()` 查询频率 |

`mcl_get_telemetry()` 的当前实现限制：`position_rad` 实际返回控制电角度，不是机械多圈位置；`ibus/temp_motor/temp_fet` 当前填 0，不能解释成真实测量值；`est_phase` 返回控制相位，不是独立的原始 SMO 相位。查看原始 SMO 输出应使用 SMO diagnostics。

## 7. 校验与维护说明

`mcl_config_validate()` 当前检查非零极对数、频率和分频，`max_duty∈(0,1]`、Lq>0、额定电流>0、过压>欠压、正且有限的时间基值、非负有限的状态机时间/回算时间，以及 AVS 组合关系。定点还检查积分周期与外环离散 Ki 的可表示范围，以及 AVS 恢复步进不能量化成零。它并非所有字段的完整物理可行性校验；“通过校验”不等于参数适合该电机。

修改参数后应通过 getter 回读确认。本文列出的“当前板级值”不能替代设备回读。维护时同时检查 `mcl_config_default()`、`drv_motor_config_default()`、`mcl_set_config()` 的采纳字段和相关 getter；新增字段应补充含义、单位、默认值与生效时机。

附录按当前公开头文件逐项列出结构体声明，用于查找类型、数组长度、条件字段及未暴露到宿主的状态。声明本身不赋予宿主直接访问权限；具体语义以以上表格和实现为准。

## 附录 A：公开结构体字段索引
以下声明按头文件提取，保留条件编译行。逗号分隔表示同类型的多个字段；函数指针为注入接口。

### `mcl`

来源：[mcl.h](../include/mcl.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_config cfg;` |
| `mcl_mode mode;` |
| `mcl_state state;` |
| `mcl_fault fault;` |
| `const mcl_hal_ops *hal;` |
| `void *hal_ctx;` |
| `mcl_foc foc;` |
| `#ifndef MCL_DISABLE_BLDC` |
| `mcl_bldc_comm bldc;` |
| `#endif` |
| `#ifndef MCL_DISABLE_OBSERVER` |
| `mcl_observer observer;` |
| `mcl_pll pll;` |
| `#endif` |
| `mcl_pid pid_speed;` |
| `mcl_pid pid_pos;` |
| `mcl_mtpa_fw mtpa_fw;` |
| `mcl_protection protection;` |
| `mcl_ctrl_mode ctrl_mode;` |
| `mcl_scalar iq_ref;` |
| `mcl_scalar speed_ref_rpm;` |
| `mcl_scalar speed_ramp_rpm;` |
| `mcl_scalar pos_ref_rad;` |
| `mcl_scalar phase_rad;` |
| `mcl_scalar speed_rad_s;` |
| `mcl_scalar fb_speed_filt;` |
| `mcl_scalar vbus;` |
| `mcl_scalar avs_scale;` |
| `mcl_scalar avs_recovery_step;` |
| `mcl_scalar speed_aw_gain;` |
| `mcl_scalar iq_applied;` |
| `mcl_scalar id_now;` |
| `mcl_scalar id_cmd;` |
| `mcl_scalar iq_now;` |
| `mcl_scalar duty_now;` |
| `mcl_scalar v_alpha_prev;` |
| `mcl_scalar v_beta_prev;` |
| `mcl_scalar dt;` |
| `mcl_scalar openloop_speed;` |
| `mcl_scalar openloop_angle;` |
| `mcl_scalar openloop_phase;` |
| `mcl_scalar openloop_mag;` |
| `float ol_timer;` |
| `float ol_hyst_timer;` |
| `float ol_anchor_timer;` |
| `mcl_scalar ol_speed;` |
| `mcl_scalar ol_phase;` |
| `float switch_blend_timer;` |
| `mcl_scalar switch_blend_iq0;` |
| `mcl_scalar switch_phase_offset;` |
| `float ol_lock_timer;` |
| `float ol_wait_timer;` |
| `uint8_t ol_stage;` |
| `uint8_t ol_started_once;` |
| `uint32_t tick_count;` |

### `mcl_motor_parameters`

来源：[mcl.h](../include/mcl.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar phase_resistance, phase_inductance, ld_lq_diff, bemf_const;` |

### `mcl_diagnostics`

来源：[mcl.h](../include/mcl.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_state state;` |
| `mcl_fault fault;` |
| `mcl_mode mode;` |
| `mcl_ctrl_mode ctrl_mode;` |
| `uint32_t tick_count;` |
| `uint8_t ol_stage;` |
| `uint32_t pole_pairs;` |
| `mcl_scalar phase_rad, speed_rad_s, iq_now, id_now, iq_ref, speed_ref_rpm;` |
| `mcl_scalar vbus, duty_now, v_alpha_prev, v_beta_prev, pll_last_phase;` |
| `mcl_scalar pos_ref_rad, openloop_mag, openloop_speed, openloop_angle;` |

### `mcl_bldc_comm`

来源：[mcl_bldc_comm.h](../include/mcl_bldc_comm.h)

| 字段声明 / 编译条件 |
|---|
| `uint8_t step;` |
| `uint8_t hall_map[8];` |
| `bool invert;` |
| `mcl_scalar bemf_integrator;` |
| `mcl_scalar bemf_threshold;` |

### `mcl_config`

来源：[mcl_config.h](../include/mcl_config.h)

| 字段声明 / 编译条件 |
|---|
| `uint8_t pole_pairs;` |
| `mcl_scalar phase_resistance;` |
| `mcl_scalar phase_inductance;` |
| `mcl_scalar ld_lq_diff;` |
| `mcl_scalar bemf_const;` |
| `mcl_scalar rated_current;` |
| `mcl_scalar rated_speed_rpm;` |
| `uint32_t pwm_freq_hz;` |
| `uint32_t current_loop_freq_hz;` |
| `uint8_t speed_loop_divider;` |
| `uint8_t pos_loop_divider;` |
| `mcl_scalar max_duty;` |
| `mcl_scalar bus_voltage;` |
| `mcl_scalar time_base;` |
| `mcl_pid_params current_pid;` |
| `mcl_pid_params speed_pid;` |
| `mcl_scalar speed_aw_time;` |
| `mcl_scalar speed_ramp_rpm_s;` |
| `mcl_pid_params pos_pid;` |
| `bool avs_enabled;` |
| `mcl_scalar avs_start_voltage;` |
| `mcl_scalar avs_stop_voltage;` |
| `mcl_scalar avs_recovery_time;` |
| `mcl_scalar avs_speed_deadband;` |
| `mcl_feedback_cfg feedback;` |
| `mcl_scalar pll_kp;` |
| `mcl_scalar pll_ki;` |
| `mcl_scalar openloop_rpm;` |
| `mcl_scalar openloop_min_rpm;` |
| `mcl_scalar openloop_rpm_low;` |
| `float openloop_hyst;` |
| `float openloop_time_lock;` |
| `float openloop_time_ramp;` |
| `float openloop_time;` |
| `mcl_scalar openloop_boost_q;` |
| `mcl_scalar openloop_max_q;` |
| `mcl_scalar openloop_drag_q;` |
| `mcl_scalar openloop_seed_angle;` |
| `mcl_protection_limits limits;` |
| `float fault_stop_time;` |
| `mcl_scalar current_offset[3];` |

### `mcl_foc`

来源：[mcl_foc.h](../include/mcl_foc.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_pid pid_d;` |
| `mcl_pid pid_q;` |
| `mcl_mtpa_fw mtpa_fw;` |
| `mcl_scalar phase_resistance;` |
| `mcl_scalar phase_inductance;` |

### `mcl_hal_ops`

来源：[mcl_hal.h](../include/mcl_hal.h)

| 字段声明 / 编译条件 |
|---|
| `void (*pwm_set_duty)(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc);` |
| `int (*adc_read_phase)(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic);` |
| `int (*adc_read_bus)(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus);` |
| `int (*enc_read_angle)(void *ctx, mcl_scalar *angle_rad);` |
| `int (*enc_read_speed)(void *ctx, mcl_scalar *speed_rad_s);` |
| `int (*read_hall)(void *ctx, uint8_t *hall);` |
| `int (*adc_read_phase_voltage)(void *ctx, mcl_scalar *va, mcl_scalar *vb, mcl_scalar *vc);` |
| `uint32_t (*micros)(void *ctx);` |
| `int (*read_temp)(void *ctx, mcl_scalar *temp_motor, mcl_scalar *temp_fet);` |

### `mcl_mtpa_fw`

来源：[mcl_mtpa_fw.h](../include/mcl_mtpa_fw.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar ld;` |
| `mcl_scalar lq;` |
| `mcl_scalar lambda;` |
| `mcl_scalar i_max;` |
| `mcl_scalar fw_id_min;` |

### `mcl_observer_ops`

来源：[mcl_observer.h](../include/mcl_observer.h)

| 字段声明 / 编译条件 |
|---|
| `void (*init)(void *impl, const void *params);` |
| `void (*reset)(void *impl);` |
| `void (*update)(void *impl, mcl_scalar v_alpha, mcl_scalar v_beta, mcl_scalar i_alpha, mcl_scalar i_beta, mcl_scalar dt, mcl_scalar *phase_rad, mcl_scalar *speed_rad_s);` |
| `void (*seed)(void *impl, mcl_scalar flux_alpha, mcl_scalar flux_beta);` |
| `mcl_scalar (*get_confidence)(void *impl);` |

### `mcl_observer`

来源：[mcl_observer.h](../include/mcl_observer.h)

| 字段声明 / 编译条件 |
|---|
| `const mcl_observer_ops *ops;` |
| `void *impl;` |
| `void *params;` |

### `mcl_observer_flux_params`

来源：[mcl_observer_flux.h](../include/mcl_observer_flux.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar lambda;` |
| `mcl_scalar resistance;` |
| `mcl_scalar inductance;` |
| `mcl_scalar gain;` |
| `mcl_scalar ld;` |

### `mcl_observer_flux`

来源：[mcl_observer_flux.h](../include/mcl_observer_flux.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_observer_flux_params params;` |
| `mcl_scalar x1;` |
| `mcl_scalar x2;` |
| `mcl_scalar lambda_est;` |
| `mcl_scalar i_alpha_last;` |
| `mcl_scalar i_beta_last;` |

### `mcl_observer_ortega_params`

来源：[mcl_observer_ortega.h](../include/mcl_observer_ortega.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar lambda;` |
| `mcl_scalar resistance;` |
| `mcl_scalar inductance;` |
| `mcl_scalar gain;` |
| `mcl_scalar ld;` |

### `mcl_observer_ortega`

来源：[mcl_observer_ortega.h](../include/mcl_observer_ortega.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_observer_ortega_params params;` |
| `mcl_scalar x1;` |
| `mcl_scalar x2;` |
| `mcl_scalar lambda_est;` |
| `mcl_scalar i_alpha_last;` |
| `mcl_scalar i_beta_last;` |
| `mcl_scalar r_est;` |
| `mcl_scalar r_est_state;` |
| `mcl_scalar speed;` |

### `mcl_observer_smo_params`

来源：[mcl_observer_smo.h](../include/mcl_observer_smo.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar resistance;` |
| `mcl_scalar inductance;` |
| `mcl_scalar flux;` |
| `mcl_scalar gain;` |
| `mcl_scalar lpf;` |
| `mcl_scalar boundary;` |
| `mcl_scalar ld;` |

### `mcl_observer_smo`

来源：[mcl_observer_smo.h](../include/mcl_observer_smo.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_observer_smo_params params;` |
| `mcl_scalar i_alpha_hat;` |
| `mcl_scalar i_beta_hat;` |
| `mcl_scalar e_alpha;` |
| `mcl_scalar e_beta;` |
| `mcl_scalar e_alpha_final;` |
| `mcl_scalar e_beta_final;` |
| `mcl_scalar w_est;` |
| `mcl_scalar theta_prev;` |
| `mcl_scalar dtheta_prev;` |
| `mcl_scalar dt;` |
| `mcl_scalar z_alpha;` |
| `mcl_scalar z_beta;` |
| `mcl_scalar seed_omega;` |
| `mcl_scalar filter_step;` |
| `mcl_scalar phase;` |
| `mcl_scalar i_alpha_last;` |
| `#if defined(MCL_USE_Q15)` |
| `int32_t filter_remainder[6];` |
| `#endif` |
| `mcl_scalar i_beta_last;` |

### `mcl_observer_smo_diagnostics`

来源：[mcl_observer_smo.h](../include/mcl_observer_smo.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar phase, e_alpha_final, e_beta_final, i_alpha_hat, i_beta_hat, w_est, z_alpha;` |

### `mcl_pid`

来源：[mcl_pid.h](../include/mcl_pid.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_pid_params params;` |
| `mcl_scalar i_term;` |
| `mcl_scalar prev_error;` |
| `mcl_scalar prev_out;` |
| `#if defined(MCL_USE_Q15) \|\| defined(MCL_USE_Q31)` |
| `int64_t aw_remainder;` |
| `int64_t aw_tracking_remainder;` |
| `#endif` |

### `mcl_pll`

来源：[mcl_pll.h](../include/mcl_pll.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar kp;` |
| `mcl_scalar ki;` |
| `mcl_scalar phase;` |
| `mcl_scalar speed;` |
| `mcl_scalar last_phase;` |
| `mcl_scalar speed_est_fast;` |
| `#if defined(MCL_USE_Q15)` |
| `int64_t phase_remainder;` |
| `int64_t speed_remainder;` |
| `#endif` |

### `mcl_protection_sample`

来源：[mcl_protection.h](../include/mcl_protection.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar ia, ib, ic;` |
| `mcl_scalar vbus, temp, speed, current;` |
| `uint32_t tick;` |

### `mcl_protection`

来源：[mcl_protection.h](../include/mcl_protection.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_protection_limits limits;` |
| `mcl_protection_status status;` |
| `mcl_fault fault;` |
| `mcl_fault_info info;` |
| `float recovery_time_s;` |
| `float recovery_elapsed_s;` |

### `mcl_pid_params`

来源：[mcl_types.h](../include/mcl_types.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar kp;` |
| `mcl_scalar ki;` |
| `mcl_scalar kd;` |
| `mcl_scalar out_min;` |
| `mcl_scalar out_max;` |
| `mcl_scalar i_min;` |
| `mcl_scalar i_max;` |

### `mcl_protection_limits`

来源：[mcl_types.h](../include/mcl_types.h)

| 字段声明 / 编译条件 |
|---|
| `uint32_t enabled;` |
| `mcl_scalar overcurrent;` |
| `mcl_scalar overvoltage;` |
| `mcl_scalar undervoltage;` |
| `mcl_scalar temp_derate_start;` |
| `mcl_scalar overtemp;` |
| `mcl_scalar abs_overcurrent;` |
| `mcl_scalar overtemp_fet;` |
| `mcl_scalar overtemp_motor;` |
| `mcl_scalar gate_overvoltage;` |
| `mcl_scalar gate_undervoltage;` |
| `mcl_scalar sincos_min;` |
| `mcl_scalar sincos_max;` |
| `mcl_scalar offset_max;` |
| `mcl_scalar unbalanced_max;` |
| `mcl_scalar overspeed;` |
| `mcl_scalar underspeed;` |
| `mcl_scalar abs_overspeed;` |

### `mcl_protection_status`

来源：[mcl_types.h](../include/mcl_types.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar temp_fet;` |
| `mcl_scalar temp_motor;` |
| `mcl_scalar gate_voltage;` |
| `mcl_scalar sincos_amplitude;` |
| `mcl_scalar current_offset[3];` |
| `uint8_t drv_fault;` |
| `uint8_t brake_fault;` |
| `uint8_t resolver_lot;` |
| `uint8_t resolver_dos;` |
| `uint8_t resolver_los;` |

### `mcl_feedback_cfg`

来源：[mcl_types.h](../include/mcl_types.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_feedback_type type;` |
| `mcl_scalar encoder_offset;` |
| `uint32_t encoder_cpr;` |

### `mcl_telemetry`

来源：[mcl_types.h](../include/mcl_types.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_scalar speed_rpm;` |
| `mcl_scalar position_rad;` |
| `mcl_scalar iq;` |
| `mcl_scalar id;` |
| `mcl_scalar vbus;` |
| `mcl_scalar ibus;` |
| `mcl_scalar duty;` |
| `mcl_scalar temp_motor;` |
| `mcl_scalar temp_fet;` |
| `mcl_scalar est_phase;` |
| `mcl_scalar est_speed_rad_s;` |

### `mcl_fault_info`

来源：[mcl_types.h](../include/mcl_types.h)

| 字段声明 / 编译条件 |
|---|
| `mcl_fault fault;` |
| `mcl_scalar current;` |
| `mcl_scalar voltage;` |
| `mcl_scalar speed;` |
| `mcl_scalar temp;` |
| `uint32_t tick;` |

## 附录 B：枚举与保护使能位

故障枚举值与使能位是两套编号，不能互相直接移位换算。

### `mcl_mode`

```c
MCL_MODE_FOC_SENSORED = 0,  /**< FOC 有感 */
    MCL_MODE_FOC_SENSORLESS,    /**< FOC 无感 */
    MCL_MODE_BLDC_HALL,         /**< 六步方波（霍尔换相） */
    MCL_MODE_BLDC_SENSORLESS,   /**< 六步方波（无感 BEMF） */
```

### `mcl_state`

```c
MCL_STATE_IDLE = 0,         /**< 未启动 */
    MCL_STATE_ALIGN,            /**< 对齐 / 校准 */
    MCL_STATE_RUN,              /**< 正常运行 */
    MCL_STATE_FAULT,            /**< 故障 */
```

### `mcl_fault`

```c
MCL_FAULT_NONE = 0,
    MCL_FAULT_OVERCURRENT,      /**< 过流 */
    MCL_FAULT_OVERVOLTAGE,      /**< 过压 */
    MCL_FAULT_UNDERVOLTAGE,     /**< 欠压 */
    MCL_FAULT_OVERTEMP,         /**< 过温（电机与功率级取高者，兼容旧阈值） */
    MCL_FAULT_STALL,            /**< 堵转 */
    MCL_FAULT_DRV,              /**< 门驱故障（nFAULT） */
    MCL_FAULT_ABS_OVERCURRENT,  /**< 绝对过流 */
    MCL_FAULT_OVERTEMP_FET,     /**< 功率级过温 */
    MCL_FAULT_OVERTEMP_MOTOR,   /**< 电机过温 */
    MCL_FAULT_GATE_OVERVOLTAGE, /**< 栅极驱动过压 */
    MCL_FAULT_GATE_UNDERVOLTAGE,/**< 栅极驱动欠压 */
    MCL_FAULT_SINCOS_LOW,       /**< 正余弦幅值过低 */
    MCL_FAULT_SINCOS_HIGH,      /**< 正余弦幅值过高 */
    MCL_FAULT_OFFSET_1,         /**< 电流传感器 1 零偏过大 */
    MCL_FAULT_OFFSET_2,         /**< 电流传感器 2 零偏过大 */
    MCL_FAULT_OFFSET_3,         /**< 电流传感器 3 零偏过大 */
    MCL_FAULT_UNBALANCED,       /**< 三相电流不平衡 */
    MCL_FAULT_BRK,              /**< 制动故障 */
    MCL_FAULT_RESOLVER_LOT,     /**< 旋变跟踪丢失 */
    MCL_FAULT_RESOLVER_DOS,     /**< 旋变信号幅度异常 */
    MCL_FAULT_RESOLVER_LOS,     /**< 旋变信号丢失 */
    MCL_FAULT_OVERSPEED,        /**< 超速 */
    MCL_FAULT_UNDERSPEED,       /**< 欠速 */
    MCL_FAULT_ABS_OVERSPEED,    /**< 绝对超速 */
```

### `mcl_feedback_type`

```c
MCL_FEEDBACK_NONE = 0,      /**< 无反馈（无感） */
    MCL_FEEDBACK_ENCODER,       /**< ABI / SPI 编码器 */
    MCL_FEEDBACK_HALL,          /**< 霍尔 */
```

### `mcl_err`

```c
MCL_OK = 0,
    MCL_ERR_PARAM,              /**< 参数非法 */
    MCL_ERR_STATE,              /**< 状态不允许 */
    MCL_ERR_HAL,                /**< HAL 返回错误 */
    MCL_ERR_BUSY,               /**< 忙（校准进行中） */
```

### `mcl_ctrl_mode`

```c
MCL_CTRL_CURRENT = 0,       /**< 电流环（直接设定 Iq） */
    MCL_CTRL_SPEED,             /**< 速度环（级联于电流环） */
    MCL_CTRL_POSITION,          /**< 位置环（级联于速度环） */
    MCL_CTRL_OPENLOOP_VF,       /**< 开环旋转电压矢量（V/F，全开环） */
    MCL_CTRL_OPENLOOP_IF,       /**< 开环旋转电流矢量（I/F，电流环闭环、相位开环） */
    MCL_CTRL_OPENLOOP_ALIGN,    /**< 开环固定电流矢量（转子预定位） */
```

### 保护位定义

```c
#define MCL_PROTECT_OVERCURRENT    (1u << 0)  /**< 过流 */
#define MCL_PROTECT_OVERVOLTAGE    (1u << 1)  /**< 过压 */
#define MCL_PROTECT_UNDERVOLTAGE   (1u << 2)  /**< 欠压 */
#define MCL_PROTECT_OVERTEMP       (1u << 3)  /**< 过温（含降额） */
#define MCL_PROTECT_ABS_OVERCURRENT (1u << 5) /**< 绝对过流 */
#define MCL_PROTECT_OVERTEMP_FET   (1u << 6)  /**< 功率级过温 */
#define MCL_PROTECT_OVERTEMP_MOTOR (1u << 7)  /**< 电机过温 */
#define MCL_PROTECT_GATE_OV        (1u << 8)  /**< 栅极驱动过压 */
#define MCL_PROTECT_GATE_UV        (1u << 9)  /**< 栅极驱动欠压 */
#define MCL_PROTECT_DRV            (1u << 10) /**< 门驱故障 */
#define MCL_PROTECT_SINCOS_LOW     (1u << 11) /**< 正余弦幅值过低 */
#define MCL_PROTECT_SINCOS_HIGH    (1u << 12) /**< 正余弦幅值过高 */
#define MCL_PROTECT_OFFSET         (1u << 13) /**< 三相电流零偏过大 */
#define MCL_PROTECT_UNBALANCED     (1u << 14) /**< 三相电流不平衡 */
#define MCL_PROTECT_BRK            (1u << 15) /**< 制动故障 */
#define MCL_PROTECT_RESOLVER       (1u << 16) /**< 旋变 LOT/DOS/LOS */
#define MCL_PROTECT_OVERSPEED      (1u << 17) /**< 超速 */
#define MCL_PROTECT_UNDERSPEED     (1u << 18) /**< 欠速 */
#define MCL_PROTECT_ABS_OVERSPEED  (1u << 19) /**< 绝对超速 */
/* 不含新增项。新增项默认关，避免没有传感器时零输入误报。 */
#define MCL_PROTECT_ALL            (MCL_PROTECT_OVERCURRENT | MCL_PROTECT_OVERVOLTAGE | \
                                    MCL_PROTECT_UNDERVOLTAGE | MCL_PROTECT_OVERTEMP)
```
