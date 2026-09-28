# Q31 板级适配与验证

当前工程已按用户要求切回 float（移除 MCL_USE_Q31/MCL_USE_Q15）；当前全局与 MCL 分组均使用 O0。以下保留 Q31 适配与验证记录。
2026-09-28 最终 Q31 全量编译：0 errors、0 warnings；未烧录、未启动实机。
本轮也对 float 和 Q15 进行了整板编译检查，均为 0 errors、0 warnings。

## 配置入口

- 板级参数：`drivers/motor/include/drv_motor_config.h`。
- 统一基值：`drivers/motor/include/drv_motor_units.h`。
- 编译期常量：`MCL_CONFIG_VALUE(physical_value, base)`，float 保留物理值，定点编码 physical_value/base。
- 动态输入：`mcl_from_physical(value, base)`；监控输出：`mcl_to_physical(value, base)`。
- 极对数、频率、计数、枚举、bool 不做 Q 格式转换。
- 开环和故障计时字段采用 float 物理秒，例如 `openloop_time_ramp = 1.5f`。
- AVS 恢复/速度回算时间仍为 mcl_scalar，使用基值 1，定点下须小于 1 秒。

基值 V=128V、I=32A、W=2048rad/s，R=V/I、L=R/W、磁链=V/W；
机械 RPM_BASE=W*60/(2*pi*5)。三种精度必须使用一致的库、驱动和应用编译宏。
参数或结构体布局改变后需要全量重编译，不能混用旧库。

## 已处理

物理常量及 ADC、母线、控制指令、PWM、监控、板级 R/L 结果转换；
固定点 FOC 前馈和电压调制换算；速度/位置外环积分周期表示；
Clarke 中间量和角度累加的提前饱和；转矩到电流公式；
Q15 SMO 滤波、PLL 和 PI 积分的小数余量；零漂校准的宽位累加平均。

## 主机回归

`tests/run_sensorless_regression.ps1` 包含原 float 无感/保护/故障恢复测试、
三精度常量测试、定点 AVS 测试和使用实际板级配置的三精度电机模型测试。
全部通过。新增板级模型测试要求稳态转速误差 <1%、角度平均误差 <10°、
仅一次开闭环切换、切换跳角 <0.1rad、PWM 合法、无意外故障。

| 场景 | float | Q15 | Q31 |
|---|---:|---:|---:|
| 带载目标 800rpm，初角 0 | 800.0 | 800.0 | 800.0 |
| 空载目标 300rpm | 300.0 | 299.9 | 300.0 |
| 带载目标 -800rpm | -800.0 | -800.0 | -800.0 |

同时覆盖不同初始角、双倍惯量、20mA 采样扰动、锁转子不误切闭环、
常量/动态单位转换、Clarke 中间量、转矩换算、4096 点正负小偏置校准。
锁转子继续开环是原有策略，本测试不代表新增堵转保护。
主机 GCC 仍会报告原有 protection 未使用 dt 参数等警告，不能混同 Keil 的零警告结果。

## 验证范围

这是平均 PWM 电机模型和软件测试，未包含真实逆变器死区、ADC 延迟、母线储能模型，
不能据此承诺实机波形、回馈峰值或实时性。当前 SMO 角度补偿、部分边界与状态机仍含浮点，
选择 Q31 并不等于整条 ISR 纯整数或一定更快。16kHz 中断耗时需实机测量。
本轮验证本工程使用的无感 FOC 路径；编码器、六步换相及未使用的通用辨识接口未作整机验证。
旧 fixed_point_speed_test / smo_closed_loop_fp_test 不作为验收依据，其物理配置与判定方式需另行更新。

## Q31 上板 LED 停闪排查（2026-09-28）

用户反馈电机仍转、控制中断仍执行，尚未取得周期测量值，不能将根因认定为 HardFault 或超时。
MCL 分组独立启用 O2/时间优化，保留全局 Q31 宏继承；其他组保持原优化设置。
初始化阶段预热三角表，避免首个控制 ISR 生成 257 项 sin 表；Q31 cos 相位偏移改用无符号回绕，消除有符号溢出。

BSP 新增 DWT 非阻塞测量，覆盖 drv_motor_control_isr（未计入测量前后的 IRQ 开销）：
- g_motor_isr_cycles / g_motor_isr_max_cycles：最近 / 最大 CPU 周期数。
- g_motor_isr_budget_cycles：SystemCoreClock/current_loop_freq_hz，170MHz/16kHz=10625。
- g_motor_isr_overruns：超预算次数。
- g_motor_isr_overrun_trip：连续 3 次超预算在 RUN 状态触发关 PWM 的次数，使用 MCL_FAULT_DRV（6）。
- g_drv.motor->start_step：6 表示启动函数已到末尾；2/4 等可定位校准/辨识等待。

DWT 只用于诊断，不用于忙等待；调试暂停期间的测量不能当成正常运行耗时。
优化后最终 Keil 编译 0 errors/0 warnings，完整主机回归通过。未烧录，本次卡顿是否消除仍待上板验证。
