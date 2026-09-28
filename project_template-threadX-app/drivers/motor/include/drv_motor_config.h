#ifndef DRV_MOTOR_CONFIG_H
#define DRV_MOTOR_CONFIG_H
#include "mcl_config.h"
#include "mcl_observer_smo.h"
#include "drv_motor_units.h"

/* Board defaults shared by firmware and host motor-model tests. */
static inline void drv_motor_config_default(mcl_config *cfg, mcl_observer_smo_params *op)
{
    mcl_config_default(cfg);
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    cfg->time_base = MCL_CONFIG_VALUE(DRV_MOTOR_TIME_BASE, 1.0f);
#endif

    /* —— 电机实体参数（铭牌：极对数 5、相间电阻 0.95Ω、V_RMS 4.6V@1000rpm、
       Ld 1.45/Lq 1.6mH @1kHz，均为「相间」值）——
       星接(Y) 换算成相值：
         - 相电阻 R_ph = R_LL/2 = 0.475Ω
         - 相磁链 λ：V_ph_RMS = 4.6/√3 = 2.656V @1000rpm(机械)，
           ω_el = 1000×2π/60×5(pole_pairs) = 523.6 rad/s，
           λ = V_ph_RMS·√2/ω_el = 2.656×1.4142/523.6 ≈ 7.17mWb
         - 相电感：d 轴 ≈ Ld/2 = 0.725mH、q 轴 ≈ Lq/2 = 0.80mH（凸极差 0.075mH） */
    cfg->pole_pairs          = DRV_MOTOR_POLE_PAIRS;                /* 极对数 = 5（Poles=5，非 10） */
    cfg->phase_resistance    = MCL_CONFIG_VALUE(0.475f, DRV_MOTOR_R_BASE);            /* 相电阻 = 线 0.95/2 */
    cfg->phase_inductance    = MCL_CONFIG_VALUE(0.8e-3f, DRV_MOTOR_L_BASE);           /* 相电感 Lq（线 Lq 1.6mH/2） */
    cfg->ld_lq_diff          = MCL_CONFIG_VALUE(0.075e-3f, DRV_MOTOR_L_BASE);         /* Lq-Ld = 0.80-0.725 = 0.075mH (IPMSM) */
    cfg->bemf_const          = MCL_CONFIG_VALUE(7.17e-3f, DRV_MOTOR_FLUX_BASE);          /* 相磁链 λ≈7.17mWb（由 4.6V@1000rpm、5 对极反解） */
    cfg->rated_current       = MCL_CONFIG_VALUE(4.0f, DRV_MOTOR_I_BASE);               /* Qcur_MAX=4A */
    cfg->rated_speed_rpm     = MCL_CONFIG_VALUE(3000.0f, DRV_MOTOR_RPM_BASE);            /* 参考值，默认保留 */
    cfg->bus_voltage         = MCL_CONFIG_VALUE(24.0f, DRV_MOTOR_V_BASE);              /* VBUS=24V */

    /* —— 运行频率 —— */
    cfg->pwm_freq_hz         = 16000u;             /* 与 hal_tim1 PWM 一致 */
    cfg->current_loop_freq_hz = 16000u;            /* 电流环 = PWM 频率 */
    cfg->speed_loop_divider  = 16u;                /* 速度环 1kHz */

    /* —— 反馈：无感 —— */
    cfg->feedback.type = MCL_FEEDBACK_NONE;
    cfg->openloop_min_rpm = MCL_CONFIG_VALUE(15.0f, DRV_MOTOR_RPM_BASE);

    /* —— 电流环 PID：物理目标 Kp=0.48V/A（L=1.6mH 时穿越频率 ≈300rad/s≈48Hz，
          高于 100rpm 电频率 16.7Hz 约 3×）、Ki=48V/(A·s)（零点 100rad/s）。
          per-unit 换算 ÷(vbus/2)=12V（v_pu=1 经 SVPWM 0.5 映射对应相电压 vbus/2）：
          老工程 ÷24 是错的（比正确值小 2×，穿越仅 ~5Hz），且解耦前馈未换算，
          导致 100rpm 拖动时 id≈-0.9A 的跟踪误差、切闭环电流重定向跟不上而失步。 */
    cfg->current_pid.kp = MCL_CONFIG_VALUE(0.48f / 12.0f, (DRV_MOTOR_V_BASE / (12.0f * DRV_MOTOR_I_BASE)));           /* ≈ 0.04 */
    cfg->current_pid.ki = MCL_CONFIG_VALUE(48.0f / 12.0f, (DRV_MOTOR_V_BASE * DRV_MOTOR_W_BASE / (12.0f * DRV_MOTOR_I_BASE)));           /* ≈ 4.0 */
    cfg->current_pid.kd = MCL_CONFIG_VALUE(0.0f, 1.0f);
    cfg->current_pid.out_min = MCL_CONFIG_VALUE(-1.0f, 1.0f);
    cfg->current_pid.out_max = MCL_CONFIG_VALUE(1.0f, 1.0f);
    cfg->current_pid.i_min   = MCL_CONFIG_VALUE(-1.0f, 1.0f);
    cfg->current_pid.i_max   = MCL_CONFIG_VALUE(1.0f, 1.0f);

    /* 速度环输出为 A，误差为机械 rpm。取消对未滤波估速的差分放大；
       闭环电流上限与既有 3A 拖动一致，避免 1.5A 上限拖不动 800rpm 风扇。
       这些增益须结合实际惯量/负载继续验证。 */
    cfg->speed_pid.kp = MCL_CONFIG_VALUE(0.005f, (DRV_MOTOR_I_BASE / DRV_MOTOR_RPM_BASE));
    cfg->speed_pid.ki = MCL_CONFIG_VALUE(0.02f, (DRV_MOTOR_I_BASE * DRV_MOTOR_W_BASE / DRV_MOTOR_RPM_BASE));
    cfg->speed_pid.kd = MCL_CONFIG_VALUE(0.0f, 1.0f);
    cfg->speed_pid.out_min = MCL_CONFIG_VALUE(-3.0f, DRV_MOTOR_I_BASE);
    cfg->speed_pid.out_max = MCL_CONFIG_VALUE(3.0f, DRV_MOTOR_I_BASE);
    cfg->speed_pid.i_min   = MCL_CONFIG_VALUE(-3.0f, DRV_MOTOR_I_BASE);
    cfg->speed_pid.i_max   = MCL_CONFIG_VALUE(3.0f, DRV_MOTOR_I_BASE);
    /* 24V commissioning values, NOT a certified hardware voltage rating.
       AVS reduces regenerative Iq; rapid braking still needs an energy sink. */
    cfg->avs_enabled = true;
    cfg->avs_start_voltage = MCL_CONFIG_VALUE(28.0f, DRV_MOTOR_V_BASE);
    cfg->avs_stop_voltage = MCL_CONFIG_VALUE(30.0f, DRV_MOTOR_V_BASE);
    cfg->avs_recovery_time = MCL_CONFIG_VALUE(0.05f, 1.0f);
    cfg->avs_speed_deadband = MCL_CONFIG_VALUE(5.0f, DRV_MOTOR_W_BASE); /* electrical rad/s */
    cfg->speed_aw_time = MCL_CONFIG_VALUE(0.02f, 1.0f);
    cfg->speed_ramp_rpm_s  = MCL_CONFIG_VALUE(500.0f, DRV_MOTOR_RPM_BASE);           /* 闭环指令斜坡 500 rpm/s：300→1000 约 1.4s */
    /* PLL: wn=100rad/s (~16Hz), damping=1. The former 40/200 setting
       lagged accelerating/decelerating rotor phase enough to cause re-drag
       in the loaded plant regression. Keep speed-loop bandwidth lower. */
    cfg->pll_kp = MCL_CONFIG_VALUE(200.0f, (DRV_MOTOR_W_BASE / DRV_MOTOR_ANGLE_BASE));
    cfg->pll_ki = MCL_CONFIG_VALUE(10000.0f, (DRV_MOTOR_W_BASE * DRV_MOTOR_W_BASE / DRV_MOTOR_ANGLE_BASE));
    /* —— 无感自动开环启动参数（VESC 式：锁定 → 斜坡 → 拖动 → 切 SMO 闭环）—— */
    cfg->openloop_rpm        = MCL_CONFIG_VALUE(300.0f, DRV_MOTOR_RPM_BASE);        /* 开环只拖到 300rpm，切闭环后由速度环升到 800rpm */
    cfg->openloop_drag_q     = MCL_CONFIG_VALUE(3.0f, DRV_MOTOR_I_BASE);           /* 开环拖动 q 轴电流 3A */
    cfg->openloop_time_lock  = 0.0f;           /* 锁定对齐时间 0s（不锁定：锁定把转子吸到固定角，
                                                可能停在 180° 不稳点 → 斜坡起步即失步（实测每次
                                                上电牵入结果随机）。改为 15rpm 场频直接牵入
                                                ——应用层 IF 测试已验证的可靠方式） */
    cfg->openloop_time_ramp  = 1.5f;           /* 拖动斜坡 1.5s（原 0.3s 太陡：转子+风扇的惯量要求
                                                J·α+风扇负载超过拖动转矩上限，转子落后磁场、打滑后
                                                I/F 平均转矩≈0 再也牵不回来（实测反电动势停在
                                                0.43~0.7V = 转子 100~200rpm 爬行）。配合斜坡从 ~0
                                                起步（mcl.c 已去掉 10% 下限），1.5s → α≈279rad/s²
                                                电角，3A 下裕量充足） */
    cfg->openloop_time       = 0.05f;          /* 拖动匀速保持 0.05s（原 0.3s：斜坡结束磁场加速度突变，
                                                转子负载角摆动，I/F 无阻尼、摆动发散失步——实测斜坡
                                                段转子已到 800rpm（反电动势 2.86V），匀速段却掉到
                                                ~200rpm（反电动势 0.75V）。斜坡一结束立刻切闭环，
                                                让 PLL/速度环阻尼转子摆动） */
    cfg->openloop_seed_angle = MCL_CONFIG_VALUE(1.5708f, DRV_MOTOR_ANGLE_BASE);           /* 已弃用：SMO 路径的 seed 角度改取「拖动期已收敛的
                                                   观测器输出角」（≈转子磁链角，负载无关）——带载 I/F
                                                   的转子超前量随负载变化，固定角 0°/π/2 都无法覆盖。
                                                   字段保留供其他观测器使用。 */

    /* 无温度传感器，不启用假温度保护。SMO 不合格只继续开环拖，不报堵转。 */
    cfg->limits.enabled = MCL_PROTECT_OVERCURRENT | MCL_PROTECT_OVERVOLTAGE |
                         MCL_PROTECT_UNDERVOLTAGE | MCL_PROTECT_ABS_OVERCURRENT |
                         MCL_PROTECT_UNBALANCED |
                         MCL_PROTECT_OVERSPEED | MCL_PROTECT_ABS_OVERSPEED;
    cfg->fault_stop_time = 0.0f; /* 锁存故障，保留诊断现场，显式清故障后才能重启 */
    cfg->limits.overcurrent  = MCL_CONFIG_VALUE(4.0f, DRV_MOTOR_I_BASE);               /* 过流 4A（与 D/Qcur_MAX 对齐） */
    cfg->limits.overvoltage  = MCL_CONFIG_VALUE(60.0f, DRV_MOTOR_V_BASE);              /* VBUS_MAX=60V */
    cfg->limits.undervoltage = MCL_CONFIG_VALUE(8.0f, DRV_MOTOR_V_BASE);               /* VBUS_MIN=8V */
    cfg->limits.abs_overcurrent = MCL_CONFIG_VALUE(8.0f, DRV_MOTOR_I_BASE);
    cfg->limits.offset_max = MCL_CONFIG_VALUE(2.0f, DRV_MOTOR_I_BASE);
    cfg->limits.unbalanced_max = MCL_CONFIG_VALUE(3.0f, DRV_MOTOR_I_BASE);
    cfg->limits.overspeed = MCL_CONFIG_VALUE(1047.0f, DRV_MOTOR_W_BASE);                /* 电气 rad/s，约 2000 rpm */
    cfg->limits.abs_overspeed = MCL_CONFIG_VALUE(1100.0f, DRV_MOTOR_W_BASE);           /* 电气 rad/s，约 2100 rpm */
    cfg->limits.overtemp     = MCL_CONFIG_VALUE(80.0f, DRV_MOTOR_TEMP_BASE);              /* Temp_MAX=80℃ */
    cfg->limits.temp_derate_start = MCL_CONFIG_VALUE(80.0f, DRV_MOTOR_TEMP_BASE);         /* 简化：80℃ 开始降额 */

    /* SMO: gain in V, boundary in A. Retain 10V/0.5A: the local correction
       slope is 20 ohm and dt*(R+gain/boundary)/L ~= 1.60 (<2).
       Efinal is internally filtered (~0.316*omega*flux at steady state);
       it must not be compared directly with the full physical back-EMF. */
    op->resistance = cfg->phase_resistance;   /* 0.475Ω 相电阻 */
    op->inductance = cfg->phase_inductance;   /* Lq，0.80mH */
    op->ld         = MCL_SUB(cfg->phase_inductance, cfg->ld_lq_diff); /* Ld；差为 0 时等于 Lq */
    op->flux       = cfg->bemf_const;         /* 7.17mWb 永磁磁链 */
    op->gain       = MCL_CONFIG_VALUE(10.0f, DRV_MOTOR_V_BASE);                  /* 滑模增益 Kslide=电压上限 V，需 > ωλ_max ≈7.5V */
    op->lpf        = MCL_CONFIG_VALUE(6.28f, DRV_MOTOR_W_BASE);                  /* 最低电气转速 1Hz（=2π rad/s，滤波系数下限） */
    op->boundary   = MCL_CONFIG_VALUE(0.5f, DRV_MOTOR_I_BASE);                   /* 线性滑模区电流误差 A（=额定 4A 的 1/8） */

}
#endif
