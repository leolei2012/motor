# 宿主参数与状态接口

宿主不得读写 MCL/SMO 实例的内部字段。实例结构体仍公开大小以支持静态分配，
但配置、指令、保护输入和诊断数据统一经 setter/getter 或专用校准 API 访问。
getter 返回值拷贝，不返回内部可写指针。

初始化参数 `mcl_config`、`mcl_observer_smo_params` 是宿主自有的数据对象，
可以填字段后传给 `mcl_init`；这与直接修改 `motor.cfg` 或 `observer.params` 不同。

| 内容 | 写入 | 读取 |
|---|---|---|
| 可调配置（PID、AVS、保护限值等） | `mcl_set_config` | `mcl_get_config` |
| 电机 R、Lq、Lq-Ld、磁链 | `mcl_set_motor_parameters`（仅 IDLE） | `mcl_get_motor_parameters` |
| SMO 参数 | `mcl_observer_smo_set_params`（停止更新时） | `mcl_observer_smo_get_params` |
| 外部保护输入 | `mcl_set_protection_status` | `mcl_get_protection_status` |
| 电流零偏 | `mcl_calibrate_offset` | `mcl_get_current_offsets` |
| 电流/速度/位置及开环指令 | 既有 `mcl_set_current/speed/position/openloop_*` | 各字段的 `mcl_get_<字段名>()` |
| 运行状态/故障 | 启停、上报、清除 API | `mcl_get_state/fault/fault_info` |
| 控制频率 | 初始化配置（重新初始化才改变） | `mcl_get_control_frequency` |
| 遥测/诊断 | 由算法产生，无宿主 setter | `mcl_get_telemetry` 和单值 getter |
| SMO 诊断 | 由观测器产生，无宿主 setter | `mcl_observer_smo_get_diagnostics` |

示例：

```c
mcl_motor_parameters params;
if (mcl_get_motor_parameters(&motor, &params) == MCL_OK)
{
    params.phase_resistance = MCL_CONFIG_VALUE(0.475f, DRV_MOTOR_R_BASE);
    params.phase_inductance = MCL_CONFIG_VALUE(0.8e-3f, DRV_MOTOR_L_BASE);
    result = mcl_set_motor_parameters(&motor, &params);
}
```

电机参数 setter 先校验，失败不更新；成功同步配置、FOC、MTPA 与绑定的 SMO。
运行中或故障态返回 MCL_ERR_STATE，不支持的观测器也返回该错误，避免仅更新部分模型。
SMO 独立 setter 适合观测器单独使用或调节滑模增益等参数；修改电机实体 R/L/磁链时，
应通过电机参数 setter 同步全部控制模块。

通用库不依赖 MCU 中断指令。宿主必须将 setter、批量 getter 与控制更新串行化。
本工程 Modbus 按寄存器使用单值 getter 读取电机状态，并进行单位换算。
SMO 保留独立诊断接口；没有跨变量或跨 Modbus 半字的同周期快照保证。
ADC 控制 ISR 内直接调用库 getter；R/L 回填发生在 IDLE，控制环未运行时。

已迁移电机驱动、监控、BSP 中断诊断的 MCL/SMO 内部字段访问。
测试中的内部缓存断言用于白盒校验，不属于宿主调用示例。
当前工程保持 float + O0。三精度接口及控制回归通过，尚未烧录验证 getter 引入后的实机耗时。

## 逐变量运行状态 getter

`mcl.h` 为电机对象的全部 37 个运行时字段提供 `mcl_get_<字段名>(const mcl *self)`，直接返回原类型数值。例如：

```c
mcl_scalar iq = mcl_get_iq_now(&motor);
mcl_scalar scale = mcl_get_avs_scale(&motor);
float remaining = mcl_get_ol_timer(&motor);
mcl_ctrl_mode mode = mcl_get_ctrl_mode(&motor);
```

这些 getter 的前置条件是实例已初始化且指针非 NULL；不分配快照、不做单位转换、不屏蔽中断。定点返回值仍为对应标幺数值，计时器仍为 float 秒。需要同一周期的多个值时由宿主统一保护读取区间。

接口采用 static inline，编译器可内联；O0 下仍可能产生函数调用，因此不能保证比一次批量读取全部字段更快。按需读少量字段可避免复制整份快照。电机批量诊断接口已移除，监控和高频控制 ISR 均使用逐变量 getter。

| getter | 返回类型 |
|---|---|
| `mcl_get_ctrl_mode()` | `mcl_ctrl_mode` |
| `mcl_get_iq_ref()` | `mcl_scalar` |
| `mcl_get_speed_ref_rpm()` | `mcl_scalar` |
| `mcl_get_speed_ramp_rpm()` | `mcl_scalar` |
| `mcl_get_pos_ref_rad()` | `mcl_scalar` |
| `mcl_get_phase_rad()` | `mcl_scalar` |
| `mcl_get_speed_rad_s()` | `mcl_scalar` |
| `mcl_get_fb_speed_filt()` | `mcl_scalar` |
| `mcl_get_vbus()` | `mcl_scalar` |
| `mcl_get_avs_scale()` | `mcl_scalar` |
| `mcl_get_avs_recovery_step()` | `mcl_scalar` |
| `mcl_get_speed_aw_gain()` | `mcl_scalar` |
| `mcl_get_iq_applied()` | `mcl_scalar` |
| `mcl_get_id_now()` | `mcl_scalar` |
| `mcl_get_id_cmd()` | `mcl_scalar` |
| `mcl_get_iq_now()` | `mcl_scalar` |
| `mcl_get_duty_now()` | `mcl_scalar` |
| `mcl_get_v_alpha_prev()` | `mcl_scalar` |
| `mcl_get_v_beta_prev()` | `mcl_scalar` |
| `mcl_get_dt()` | `mcl_scalar` |
| `mcl_get_openloop_speed()` | `mcl_scalar` |
| `mcl_get_openloop_angle()` | `mcl_scalar` |
| `mcl_get_openloop_phase()` | `mcl_scalar` |
| `mcl_get_openloop_mag()` | `mcl_scalar` |
| `mcl_get_ol_timer()` | `float` |
| `mcl_get_ol_hyst_timer()` | `float` |
| `mcl_get_ol_anchor_timer()` | `float` |
| `mcl_get_ol_speed()` | `mcl_scalar` |
| `mcl_get_ol_phase()` | `mcl_scalar` |
| `mcl_get_switch_blend_timer()` | `float` |
| `mcl_get_switch_blend_iq0()` | `mcl_scalar` |
| `mcl_get_switch_phase_offset()` | `mcl_scalar` |
| `mcl_get_ol_lock_timer()` | `float` |
| `mcl_get_ol_wait_timer()` | `float` |
| `mcl_get_ol_stage()` | `uint8_t` |
| `mcl_get_ol_started_once()` | `uint8_t` |
| `mcl_get_tick_count()` | `uint32_t` |

另提供 `mcl_get_pll_last_phase()`，返回前拍 PLL 输入角；裁剪观测器时返回零。

`mcl_get_mode()` 返回运行模式，`mcl_get_pole_pairs()` 返回极对数；两者同样要求非 NULL 的已初始化实例。
