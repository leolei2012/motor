# 母线限回馈控制（AVS v1）

TI SLAU927C §7.1.11 描述 AVS 自动减缓减速，避免回馈引起电源过压；
本实现是相同目的的母线反馈转矩限幅，不是 TI 内部算法的移植。
参考：https://www.ti.com/lit/ug/slau927/slau927.pdf

## 接入与配置

支持 float/Q15/Q31 FOC 闭环速度、位置、电流模式。自动开环拖动以及手动
VF/IF/ALIGN 不受此限幅控制，仍依赖原有保护。
所有参数平铺于 mcl_config。avs_enabled 为显式开关，关闭时保留参数。
库默认关闭 AVS，float 阈值默认 27/29V（对应库默认 24V 母线、30V 硬过压）；
speed_aw_time 默认 0 保持既有控制行为，驱动设为 20ms 后独立启用速度抗饱和。
AVS 启用要求 speed_aw_time > 0，关闭 AVS 不会关闭已启用的速度抗饱和。
定点电压/速度采用标幺，时间参数 avs_recovery_time、speed_aw_time 统一用物理秒，
不除 time_base。这两个时间须小于 1s，并保证量化后非零；配置校验拒绝零恢复步长
和不能表示的归一化速度环周期。初始化/配置更新时换算无量纲系数，定点控制 tick
不使用浮点 AVS 运算。积分和回算分别保存亚 LSB 余数，中间量使用 int64_t。
例如 V_BASE=64V、W_BASE=1000rad/s：

```c
cfg.avs_enabled = true;
cfg.avs_start_voltage = MCL_FROM_FLOAT(28.0f / 64.0f);
cfg.avs_stop_voltage = MCL_FROM_FLOAT(30.0f / 64.0f);
cfg.avs_speed_deadband = MCL_FROM_FLOAT(5.0f / 1000.0f);
cfg.avs_recovery_time = MCL_FROM_FLOAT(0.05f);
cfg.speed_aw_time = MCL_FROM_FLOAT(0.02f);
```

母线额定值、硬过压和其他电机参数必须使用相同基值。V_BASE 必须高于硬过压阈值，
不能把 24V 直接作为电压基值后再表示 28/30V。这个例子不是整板定点配置，
当前 STM32 工程选用 float；驱动配置、ADC、指令、PWM 和监控边界已按统一基值转换。
三种精度通过主机板级模型测试，但未烧录验证定点版实机与中断耗时。
修改这些参数应使用 mcl_set_config（与控制中断互斥，推荐停机），不要仅在调试器
里改时间字段，否则缓存系数不会同步更新。avs_enabled 临时关闭仍可直接生效。
电机驱动对 24V 系统配置：28V 开始、30V 取消主动回馈，
50ms 恢复制动力，5rad/s 电角速度方向死区，20ms 速度积分回算时间。
这些是调试起点，不是硬件耐压认证。保留工作区已有的硬过压阈值配置。

每个电流 tick 更新 avs_scale：升压立即收紧、降压线性缓慢恢复。
正转缩小负 Iq 上限，反转缩小正 Iq 上限，近零速同时缩小两侧。
速度 PI 使用包含温度降额及 AVS 的动态限值，条件积分加回算避免饱和积累。
电流环每拍再限幅，防止速度环分频造成漏限；电流模式保留用户原始指令。
限流会延长恢复目标速度的时间，不承诺固定制动时间或位置跟踪精度。

母线采样仍为 ADC1 规则组 1kHz 非阻塞读取，检查 EOC，转换未完成不读旧结果；
控制 voltage_mv 在 1kHz ISR 发布，上升立即跟随、下降以 1/4 系数恢复。
100ms poll 仅更新慢滤波 ADC 遥测，不覆盖控制电压。PWM 干扰可能导致保守限流或误跳闸，
应在板上确认 PA3 波形及采样点；1ms 采样也不保证捕获亚毫秒尖峰。

## 验证与边界

运行 tests/run_sensorless_regression.ps1。avs_test 覆盖 FOC 接入、正反转、
方向死区、恢复、硬过压和 PI 饱和释放；这是软件行为测试，不是实机安全验证。
尚未验证真实电机、电容、回馈电源、采样延迟共同作用下的峰值电压。

烧录前核实功率板最低耐压，恢复合适的硬保护阈值，不以 100V 电容替代整板评估。
先用可控小负载，示波器同步记录板端 Vbus；监视 motor.avs_scale、motor.iq_ref、
motor.iq_applied、motor.iq_now、motor.speed_rad_s，以及 bus_voltage.ch.raw_adc、
bus_voltage.ch.missed_samples。结构体新增字段后必须重新加载对应 ELF 符号。

AVS 不吸收能量：外力持续拖动、估速失准、电流环饱和、采样失效、体二极管回馈
都可能使软件限流失效。需要快速/持续制动时必须提供硬件能量吸收通路。
