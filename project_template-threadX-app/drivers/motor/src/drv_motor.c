#include "drv_motor.h"

#include <math.h>
#include <string.h>

#include "hal_tim1.h"
#include "hal_adc1.h"
#include "hal_adc2.h"
#include "drv.h"

/*
 *  drv_motor：电机驱动，drivers 层实例化 mcl 算法库。
 *
 *  职责：
 *   - 持有 mcl 实例 + 磁链观测器实例（static，组合根注入到 g_drv.motor）
 *   - 实现 mcl_hal_ops，把 mcl 的抽象硬件回调接到 hal_tim1 / hal_adc1 / hal_adc2
 *   - 封装 mcl 面门 API 为设备级接口（start/stop/set_speed/set_current）
 *
 *  电流采样：三电阻方案，ADC1 采 U/V（注入组 rank1=U, rank2=V），
 *  ADC2 采 V/W（rank1=V, rank2=W）。电流 ADC 12bit 左对齐，零点 32768。
 */

/* ==================== 电流标定（需按实际 shunt/运放标定） ==================== */

/*
 * 电流换算：I = (adc_raw - 32768) × CURRENT_ADC_SCALE_A
 *
 * 方向约定（适配 mcl 库标准 Clarke/Park convention）：
 *   mcl 的 Clarke：i_alpha = ia，期望 ia/ib/ic 为「流入电机绕组为正」。
 *   运放同相（+接源极，-接地）：电流流入电机 → ADC 值升高 → 正值。
 *
 * 这里以 32768（12bit 左对齐量程中点）作为「参考零点」，返回原始换算电流。
 * 真实的零电流偏置（器件 1.25V ≈ raw 24824）会导致零电流时读到 ≈ -3.33A
 * 的虚假电流，由 mcl 的 current_offset 动态校准吸收：
 *   - mcl_calibrate_offset() 在零电流下采样平均，把该虚假电流存入
 *     cfg.current_offset[3]；
 *   - mcl.control_tick 每次采样后减去 current_offset，即得零偏置电流。
 * 故不需要在此硬编码偏置电压，漂移也能被动态校正。
 *
 * 标定系数（Rshunt=0.02Ω, Gain=6, Vref=3.3V）：
 *   增益 = Rf/Rin = 12K/(1K+1K) = 6（-端反馈 12K，-端两个 1K 串联到地）
 *   I = (raw - 32768) / 65536 × 3.3 / (0.02 × 6)
 *     = (raw - 32768) × 0.0004196
 */
#define CURRENT_ADC_SCALE_A     0.0004196f   /**< 3.3/(0.02×6)/65536，左对齐 A/count */

/* 量程中点（12bit 左对齐 16bit 量程的一半），作为 raw→A 的参考零点 */
#define CURRENT_ADC_MID         32768.0f

/*
 * 电流采样方向：
 *
 *  三电阻低边采样：采样电阻在绕组与 GND 之间，采到的是「流出电机」的电流，
 *  而 mcl 约定 ia/ib/ic 为「流入电机为正」。故需反向（mid - raw）才能匹配
 *  mcl 的 Clarke/Park 约定，电流环才是负反馈。
 *
 *  实证：INVERT=0 时 IF 与 SMO 闭环均一上电即硬件过流（电流环正反馈）；
 *        INVERT=1 时电流环稳定（Iq 精确跟踪设定）。
 */
#define CURRENT_ADC_INVERT     1   /**< 1=反向(mid - raw)，匹配 mcl 流入为正约定 */

/** 16kHz 控制节拍计数（ADC JEOS ISR 递增，每 62.5µs 一拍）：
    提供与 PWM/ADC 硬件同源的精确短延时，替代不可靠的 DWT。 */
static volatile uint32_t s_jeos_count = 0u;

/* ==================== mcl_hal_ops 回调 ==================== */

/** 读取三相电流（ADC 注入组，左对齐 12bit → 安培，含偏置，零漂由 mcl 校准） */
static int drv_motor_adc_read_phase(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    struct drv_motor *self = (struct drv_motor *)ctx;

    uint16_t ia_raw = hal_adc1_inj_read_jdr1();  /* U 相 (ADC1 rank1) */
    uint16_t ib_raw = hal_adc1_inj_read_jdr2();  /* V 相 (ADC1 rank2) */
    uint16_t ic_raw = hal_adc2_inj_read_jdr2();  /* W 相 (ADC2 rank2) */

#if CURRENT_ADC_INVERT
    *ia = (mcl_scalar)(((float)CURRENT_ADC_MID - (float)ia_raw) * CURRENT_ADC_SCALE_A);
    *ib = (mcl_scalar)(((float)CURRENT_ADC_MID - (float)ib_raw) * CURRENT_ADC_SCALE_A);
    *ic = (mcl_scalar)(((float)CURRENT_ADC_MID - (float)ic_raw) * CURRENT_ADC_SCALE_A);
#else
    *ia = (mcl_scalar)(((float)ia_raw - CURRENT_ADC_MID) * CURRENT_ADC_SCALE_A);
    *ib = (mcl_scalar)(((float)ib_raw - CURRENT_ADC_MID) * CURRENT_ADC_SCALE_A);
    *ic = (mcl_scalar)(((float)ic_raw - CURRENT_ADC_MID) * CURRENT_ADC_SCALE_A);
#endif

    /* 调试观测：保存减零漂后的三相电流（与 FOC 实际使用的一致） */
    if (self != NULL)
    {
        self->ia_now = (float)(*ia - self->motor.cfg.current_offset[0]);
        self->ib_now = (float)(*ib - self->motor.cfg.current_offset[1]);
        self->ic_now = (float)(*ic - self->motor.cfg.current_offset[2]);
    }

    return MCL_OK;
}

/** 读取母线电压/电流（母线电压来自 drv_ain_sensor） */
static int drv_motor_adc_read_bus(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus)
{
    (void)ctx;

    /*
     * 母线电压由 drv_ain_sensor 采样并换算成 mV，这里转成 V。
     * 启动初期 ain_sensor 尚未轮询（voltage_mv=0）：回退额定母线 24V，
     * 避免 vbus=0 使 mcl 解耦前馈 ÷(vbus/2) 除零 → ±inf/NaN → PLL 相位
     * 回绕 while 循环卡死（曾实测整机冻结）。
     * 母线电流无传感器，暂填 0。
     */
    if (g_drv.ain_sensor != NULL && g_drv.ain_sensor->bus_voltage.voltage_mv > 0u)
    {
        *vbus = (mcl_scalar)((float)g_drv.ain_sensor->bus_voltage.voltage_mv / 1000.0f);
    }
    else
    {
        *vbus = 24.0f;
    }
    *ibus = 0.0f;

    return MCL_OK;
}

/** 输出三相占空比（mcl SVPWM 输出 [0, max_duty] → hal_tim1 cmp [0, PWM_HALF_PERIOD]） */
static void drv_motor_pwm_set_duty(void *ctx, mcl_scalar da, mcl_scalar db, mcl_scalar dc)
{
    (void)ctx;

    /*
     * mcl 的 da/db/dc 是 SVPWM 输出后的相占空比，范围 [0, max_duty]（0=全低，1=全高）。
     * 直接映射到中心对齐 PWM 的比较值 [0, PWM_HALF_PERIOD]。
     */
    uint16_t cmp_a = (uint16_t)((float)da * (float)PWM_HALF_PERIOD);
    uint16_t cmp_b = (uint16_t)((float)db * (float)PWM_HALF_PERIOD);
    uint16_t cmp_c = (uint16_t)((float)dc * (float)PWM_HALF_PERIOD);

    /* 保险钳位：异常/NaN/超限输入不得写出大于 ARR 的比较值（CCR>ARR 输出常开，
       曾因垃圾占空比导致过流）。 */
    if (cmp_a > PWM_HALF_PERIOD) { cmp_a = PWM_HALF_PERIOD; }
    if (cmp_b > PWM_HALF_PERIOD) { cmp_b = PWM_HALF_PERIOD; }
    if (cmp_c > PWM_HALF_PERIOD) { cmp_c = PWM_HALF_PERIOD; }

    hal_tim1_set_duty_abc(cmp_a, cmp_b, cmp_c);
}

/** 微秒时间基准：HAL_GetTick（TIM6 1kHz 驱动）×1000 → µs。
    替代 DWT CYCCNT 版本：调试器复位后 DWT 的 TRCENA/CYCCNTENA 状态不可靠
    （CYCCNTENA 可能残留 1 而 TRCENA 被清 → CYCCNT 冻结），曾导致 R/L 实测
    忙等待全部打到 ~5s/次的上限、启动流程卡死数分钟。 */
static uint32_t drv_motor_micros(void *ctx)
{
    (void)ctx;
    return HAL_GetTick() * 1000u;
}

/** 读温度（无传感器，返回 0） */
static int drv_motor_read_temp(void *ctx, mcl_scalar *temp_motor, mcl_scalar *temp_fet)
{
    (void)ctx;
    *temp_motor = 0.0f;
    *temp_fet   = 0.0f;
    return MCL_OK;
}

static const mcl_hal_ops s_mcl_hal =
{
    .pwm_set_duty        = drv_motor_pwm_set_duty,
    .adc_read_phase      = drv_motor_adc_read_phase,
    .adc_read_bus        = drv_motor_adc_read_bus,
    .enc_read_angle      = NULL,   /* 无感，不提供 */
    .enc_read_speed      = NULL,
    .read_hall           = NULL,
    .adc_read_phase_voltage = NULL,
    .micros              = drv_motor_micros,
    .read_temp           = drv_motor_read_temp,
};

/* ==================== 初始化 ==================== */

int drv_motor_init(struct drv_motor *self)
{
    if (self == NULL)
    {
        return -1;
    }

    memset(self, 0, sizeof(*self));

    /* 电机配置：从 motor_control-v2.0 的 UserData_Motor.h / UserData_Parameter.h 抄录 */
    mcl_config cfg;
    mcl_config_default(&cfg);

    /* —— 电机实体参数（铭牌：极对数 5、相间电阻 0.95Ω、V_RMS 4.6V@1000rpm、
       Ld 1.45/Lq 1.6mH @1kHz，均为「相间」值）——
       星接(Y) 换算成相值：
         - 相电阻 R_ph = R_LL/2 = 0.475Ω
         - 相磁链 λ：V_ph_RMS = 4.6/√3 = 2.656V @1000rpm(机械)，
           ω_el = 1000×2π/60×5(pole_pairs) = 523.6 rad/s，
           λ = V_ph_RMS·√2/ω_el = 2.656×1.4142/523.6 ≈ 7.17mWb
         - 相电感：d 轴 ≈ Ld/2 = 0.725mH、q 轴 ≈ Lq/2 = 0.80mH（凸极差 0.075mH） */
    cfg.pole_pairs          = 5u;                /* 极对数 = 5（Poles=5，非 10） */
    cfg.phase_resistance    = 0.475f;            /* 相电阻 = 线 0.95/2 */
    cfg.phase_inductance    = 0.8e-3f;           /* 相电感 Lq（线 Lq 1.6mH/2） */
    cfg.ld_lq_diff          = 0.075e-3f;         /* Lq-Ld = 0.80-0.725 = 0.075mH (IPMSM) */
    cfg.bemf_const          = 7.17e-3f;          /* 相磁链 λ≈7.17mWb（由 4.6V@1000rpm、5 对极反解） */
    cfg.rated_current       = 4.0f;               /* Qcur_MAX=4A */
    cfg.rated_speed_rpm     = 3000.0f;            /* 参考值，默认保留 */
    cfg.bus_voltage         = 24.0f;              /* VBUS=24V */

    /* —— 运行频率 —— */
    cfg.pwm_freq_hz         = 16000u;             /* 与 hal_tim1 PWM 一致 */
    cfg.current_loop_freq_hz = 16000u;            /* 电流环 = PWM 频率 */
    cfg.speed_loop_divider  = 16u;                /* 速度环 1kHz */

    /* —— 反馈：无感 —— */
    cfg.feedback.type = MCL_FEEDBACK_NONE;

    /* —— 电流环 PID：物理目标 Kp=0.48V/A（L=1.6mH 时穿越频率 ≈300rad/s≈48Hz，
          高于 100rpm 电频率 16.7Hz 约 3×）、Ki=48V/(A·s)（零点 100rad/s）。
          per-unit 换算 ÷(vbus/2)=12V（v_pu=1 经 SVPWM 0.5 映射对应相电压 vbus/2）：
          老工程 ÷24 是错的（比正确值小 2×，穿越仅 ~5Hz），且解耦前馈未换算，
          导致 100rpm 拖动时 id≈-0.9A 的跟踪误差、切闭环电流重定向跟不上而失步。 */
    cfg.current_pid.kp = 0.48f / 12.0f;           /* ≈ 0.04 */
    cfg.current_pid.ki = 48.0f / 12.0f;           /* ≈ 4.0 */
    cfg.current_pid.kd = 0.0f;
    cfg.current_pid.out_min = -1.0f;
    cfg.current_pid.out_max = 1.0f;
    cfg.current_pid.i_min   = -1.0f;
    cfg.current_pid.i_max   = 1.0f;

    /* —— 速度环 PID：mcl 的速度环误差量纲是 rpm（fb_rpm），老工程 0.5/2.0
          是按 A/(rad/s) 整定的，直接用于 rpm 大 9.55× → 切闭环 50rpm 误差瞬间
          把 iq_ref 打到 ±1.5A 饱和 → 加速→超调(实测冲到 191rpm)→急刹→低速→
          再饱和，形成 1-2Hz 大摆幅极限环。折算到 rpm 并再调柔：
          Kp=0.02 A/rpm（50rpm 误差→1.0A，穿越 ~6Hz）、Ki=0.1（零点 ~0.8Hz）。 */
    cfg.speed_pid.kp = 0.005f;
    cfg.speed_pid.ki = 0.002f;   /* 恢复小 ki：纯 PD(ki=0) 时 i_term 预置 1A 残留成永久偏置，
                                    把转速顶到 480rpm 而非 300。ki=0.002 让 i_term 收敛到正确
                                    稳态值(iq≈0.3A)，消静差又不致积分慢摆。 */
    cfg.speed_pid.kd = 0.5f;     /* 微分阻尼：消除 195~352rpm 的剩余摆动。d_term=kd×(err−prev_err)，
                                    未除 dt(1ms)，摆动 ±80rpm/周期~4s → 每ms误差变~0.08rpm，
                                    kd=0.5 → ~0.04A 阻尼（温和）。 */
    cfg.speed_pid.out_min = -1.5f;
    cfg.speed_pid.out_max = 1.5f;
    cfg.speed_pid.i_min   = -1.5f;
    cfg.speed_pid.i_max   = 1.5f;

    /* —— PLL（kp 直接耦合输入相位噪声到相位速率 kp·err：kp=80 时 0.5rad 抖动
          → ±40rad/s 帧速率摆动，电流矢量被甩来甩去。VESC 式低 kp 高阻尼折中：
          kp=40、ki=1000 → ωn≈31.6rad/s≈5Hz、ζ≈0.63，速度噪声 ≈1/8 of ki=8000。 */
    cfg.pll_kp = 40.0f;
    cfg.pll_ki = 200.0f;  /* 切闭环后 PLL 速度估计超调（实测冲到 400rpm=209rad/s，真实转子仅 300rpm），
                            速度环据此反向加大 iq 刹停→掉速→重开环极限环。降到 200 让速度估计
                            更平滑，减小与真实转速的偏差（对齐 VESC 低速低带宽）。 */

    /* —— 无感自动开环启动参数（VESC 式：锁定 → 斜坡 → 拖动 → 切 SMO 闭环）—— */
    cfg.openloop_rpm        = 300.0f;        /* 开环拖动转速上限 300rpm（切闭环反电动势充足）。
                                                历史教训：50rpm 时反电动势 ωλ=2.6×7.17mWb≈0.19V，
                                                相对电阻压降 R·i≈0.95V（2A）信噪仅 1:5，Ortega 纯积分
                                                观测器角度被 R·i 主导 → 轻微 R 误差就让磁链相位漂移
                                                ~50rad/s、~64ms 内 180° 翻转失锁。300rpm 反电动势
                                                ≈1.13V 与 R·i 相当，观测器有足够信号锁定。 */
    cfg.openloop_drag_q     = 2.0f;           /* 开环拖动 q 轴电流 2A（锁定与拖动共用，加强对齐） */
    cfg.openloop_time_lock  = 0.2f;           /* 锁定对齐时间 0.2s（加长，确保转子可靠对齐到稳定平衡点） */
    cfg.openloop_time_ramp  = 0.3f;           /* 拖动斜坡 0.3s（原默认 0.1s 太短：转子从锁定位
                                                到拖动位要摆 ~90°，摆动未稳就切闭环 → 帧超前真实
                                                磁链 → 负转矩急停。加长让摆动衰减） */
    cfg.openloop_time       = 0.3f;           /* 拖动匀速保持 0.3s（原默认 0.05s，同上加长等转子稳定） */
    cfg.openloop_seed_angle = 0.78539816f;    /* seed 负载角补偿 = π/4 = 45°（对齐 VESC foc 低速段
                                                 seed：phase + SIGN(duty)*M_PI/4）。历史教训：曾用 0°
                                                 （无补偿，观测器从磁场角起步 → 磁链方向超前真实转子 →
                                                 PLL 估速偏高 ~30% → 速度环慢摆/低速失锁）；也曾按
                                                 「转子超前 90°」误 seed 反电动势方向导致 100ms 必崩。
                                                 VESC 实测用 45° 折中：既补偿 I/F 拖动滑差，又不致 90°
                                                 完全错位。 */

    /* —— 保护阈值（UserData_Motor.h safe 段）—— */
    /*
     * 开环 VF 调试阶段：先禁用所有保护，排除保护误触发导致的"电机不转"。
     * 验证 PWM 输出 + 接线正确后再恢复保护（enabled = MCL_PROTECT_ALL）。
     */
    cfg.limits.enabled      = 0u;                 /* TODO: 调试期禁用，验证后恢复 */
    cfg.limits.overcurrent  = 4.0f;               /* 过流 4A（与 D/Qcur_MAX 对齐） */
    cfg.limits.overvoltage  = 30.0f;              /* VBUS_MAX=30V */
    cfg.limits.undervoltage = 10.0f;              /* VBUS_MIN=10V */
    cfg.limits.overtemp     = 80.0f;              /* Temp_MAX=80℃ */
    cfg.limits.temp_derate_start = 80.0f;         /* 简化：80℃ 开始降额 */
    cfg.limits.stall_speed  = 1.0f;               /* 堵转转速 rad/s（保守默认） */
    cfg.limits.stall_time   = 0.5f;               /* 堵转时间 0.5s */

    /* Ortega 磁链观测器（VESC 式，λ²−|λ|² 双向幅值反馈）参数：
       角度 = atan2(λ_β, λ_α) 直接是转子磁链角（=FOC d 轴），幅值反馈
       err·λ_r 会让 λ 自动收敛到真实磁链（初始角度/R/L 偏差都会自校正），
       没有 SMO 那种「预设相移 δ、口径一变就翻 90°/180°」的坑。
       R/L 初值用配置值，启动后由 drv_motor_measure_rl() 实测回填。 */
    mcl_observer_ortega_params op;
    op.lambda     = cfg.bemf_const;         /* 7.17mWb 永磁磁链 */
    op.resistance = cfg.phase_resistance;   /* 0.475Ω（相值=线 0.95/2） */
    op.inductance = cfg.phase_inductance;   /* 0.80mH（相电感 Lq） */
    op.gain       = 750000.0f;               /* 观测器增益 γ = 600/L = 600/0.0008（VESC 注释推荐量级，
                                               对应 m_gamma 未 ×4）。恢复非对称 clamp 后必须回到此量级
                                               压住积分慢漂；100~180000 太弱致慢漂掉速，750000 实测最佳。 */

    if (mcl_init(&self->motor, &cfg, &s_mcl_hal, self,
                 &mcl_observer_ortega_ops, &self->observer, &op) != MCL_OK)
    {
        return -1;
    }

    mcl_set_mode(&self->motor, MCL_MODE_FOC_SENSORLESS);

    /*
     * 电流零漂校准不在此处做：init 阶段 TIM1 尚未启动，ADC 注入组未被
     * TRGO 触发，JDR 读到的都是未转换的垃圾值，校准出的 current_offset
     * 完全不可信。改在 drv_motor_start() 里「TIM1 已启动(有 TRGO 触发)、
     * MOE 未使能(电流为 0)」的正确时机采样校准。
     */

    return 0;
}

/* ==================== 封装面门 API ==================== */

int drv_motor_start(struct drv_motor *self)
{
    if (self == NULL)
    {
        return -1;
    }

    /*
     * 启动硬件时序（参考 ST MCSDK + 电流零漂校准 + R/L 实测）：
     *   1. 使能驱动器（M1_EN_DRIVER）
     *   2. 启动 TIM1 计数器（TRGO → ADC 注入组采样）。此时 mcl 尚未 start
     *      （状态 IDLE，控制节拍只采样不写 PWM），不会干扰后续测量
     *   3. 电流零漂校准（TIM1 在跑、MOE 未开、电流为 0，256 次平均）
     *   4. 使能 PWM 输出（MOE）
     *   5. R/L 实测（直流 α 注入 + 电压脉冲法，回填观测器/解耦前馈参数；
     *      best effort，失败则沿用配置参数继续跑）
     *   6. mcl start → 无感自动开环启动序列（锁定 → 斜坡 → 拖动 → 切闭环）
     */
    drv_output_set(g_drv.output, DRV_OUTPUT_EN_DRIVER, DRV_OUTPUT_ON);
    self->start_step = 1u;
    hal_tim1_start();
    self->start_step = 2u;

    if (drv_motor_calibrate_offset(self) != 0)
    {
        /* 校准失败：不输出 PWM，安全停机 */
        hal_tim1_stop();
        drv_output_set(g_drv.output, DRV_OUTPUT_EN_DRIVER, DRV_OUTPUT_OFF);
        mcl_stop(&self->motor);
        return -1;
    }
    self->start_step = 3u;

    hal_tim1_pwm_enable();
    self->start_step = 4u;

    /* R/L 实测（失败不阻断启动：沿用配置参数，r_meas/l_meas 保持 0 供诊断） */
    (void)drv_motor_measure_rl(self);
    self->start_step = 5u;

    if (mcl_start(&self->motor) != MCL_OK)
    {
        hal_tim1_pwm_disable();
        hal_tim1_stop();
        drv_output_set(g_drv.output, DRV_OUTPUT_EN_DRIVER, DRV_OUTPUT_OFF);
        return -1;
    }
    self->start_step = 6u;

    return 0;
}

int drv_motor_stop(struct drv_motor *self)
{
    if (self == NULL)
    {
        return -1;
    }

    /*
     * 关断时序（与启动相反）：
     *   1. 紧急关断 PWM 输出（全部高阻）
     *   2. 停 TIM1 计数器
     *   3. 禁用驱动器
     */
    hal_tim1_pwm_disable();
    hal_tim1_stop();
    drv_output_set(g_drv.output, DRV_OUTPUT_EN_DRIVER, DRV_OUTPUT_OFF);

    if (mcl_stop(&self->motor) != MCL_OK)
    {
        return -1;
    }

    return 0;
}

int drv_motor_set_current(struct drv_motor *self, float iq_ref)
{
    if (self == NULL)
    {
        return -1;
    }

    if (mcl_set_current(&self->motor, (mcl_scalar)iq_ref) != MCL_OK)
    {
        return -1;
    }

    return 0;
}

int drv_motor_set_speed(struct drv_motor *self, float speed_rpm)
{
    if (self == NULL)
    {
        return -1;
    }

    if (mcl_set_speed(&self->motor, (mcl_scalar)speed_rpm) != MCL_OK)
    {
        return -1;
    }

    return 0;
}

int drv_motor_set_openloop_vf(struct drv_motor *self, float voltage, float speed_rpm)
{
    if (self == NULL)
    {
        return -1;
    }

    if (mcl_set_openloop_vf(&self->motor, (mcl_scalar)voltage, (mcl_scalar)speed_rpm) != MCL_OK)
    {
        return -1;
    }

    return 0;
}

int drv_motor_set_openloop_if(struct drv_motor *self, float current, float speed_rpm)
{
    if (self == NULL)
    {
        return -1;
    }

    if (mcl_set_openloop_if(&self->motor, (mcl_scalar)current, (mcl_scalar)speed_rpm) != MCL_OK)
    {
        return -1;
    }

    return 0;
}

/** 观测器输出角（与 ortega_update 一致：atan2(λ_β, λ_α)，λ_r = x − L·i，
    即转子磁链角 = FOC d 轴，无任何固定相移/经验修正角）。 */
static float drv_observer_output_angle(struct drv_motor *self)
{
    float la = (float)self->observer.x1
             - (float)self->observer.params.inductance * (float)self->observer.i_alpha_last;
    float lb = (float)self->observer.x2
             - (float)self->observer.params.inductance * (float)self->observer.i_beta_last;
    return atan2f(lb, la);
}

/** 上一拍开环阶段（切换捕获用） */
static uint8_t s_prev_ol_stage = 0u;

/** 切换后逐拍采集：对数间隔偏移（1,2,4,...,8192 拍 @16kHz = 62.5µs~512ms） */
static const uint32_t s_cap2_off[14] = {1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u, 256u, 512u, 1024u, 2048u, 4096u, 8192u};
static uint8_t  s_cap2_idx   = 0u;   /**< 下一个待采集偏移索引 */
static uint8_t  s_cap2_armed = 0u;   /**< 1=本轮闭环采集进行中 */
static uint32_t s_cap2_tick  = 0u;   /**< 本轮回合内拍数 */

void drv_motor_control_isr(struct drv_motor *self)
{
    uint8_t ol;

    if (self == NULL)
    {
        return;
    }

    s_jeos_count++;
    mcl_control_tick(&self->motor);

    /* 切换瞬间事件捕获：拖动(2)→闭环(0) 的帧角/观测器角/速度，
       定位切闭环崩溃的相位跳变（一次性，cap_valid 置位后不再覆盖） */
    ol = (uint8_t)self->motor.ol_stage;
    if (ol == 2u)
    {
        self->cap_pre_est = (float)self->motor.phase_rad;
        self->cap_pre_lam = drv_observer_output_angle(self);
        self->cap_in_prev2 = self->cap_in_prev1;
        self->cap_in_prev1 = (float)self->motor.pll.last_phase;   /* = 本拍 seed 前观测器输出（PLL 输入） */
    }
    else if (ol == 0u && s_prev_ol_stage == 2u && self->cap_valid == 0u)
    {
        self->cap_post_est = (float)self->motor.phase_rad;
        self->cap_post_lam = drv_observer_output_angle(self);
        self->cap_post_spd = (float)self->motor.speed_rad_s;
        self->cap_pll_last = (float)self->motor.pll.last_phase;
        self->cap_valid = 1u;
    }

    /* 切换后逐拍采集（cap2）：每轮拖动→闭环重新武装，对数间隔采样 14 个点，
       重开环时锁存存活拍数。数据在开环期间静止，Modbus 读到的总是一整轮完整结果 */
    if (ol == 0u)
    {
        if (s_prev_ol_stage == 2u)
        {
            s_cap2_armed = 1u;
            s_cap2_idx = 0u;
            s_cap2_tick = 0u;
            self->cap2_switch_count++;
            self->cap2_min_spd = 1.0e9f;
            self->cap2_max_iq = 0.0f;
        }
        if (s_cap2_armed != 0u)
        {
            s_cap2_tick++;
            if (s_cap2_idx < 14u && s_cap2_tick == s_cap2_off[s_cap2_idx])
            {
                self->cap2_frame[s_cap2_idx] = (float)self->motor.phase_rad;
                self->cap2_obs[s_cap2_idx]   = drv_observer_output_angle(self);
                self->cap2_spd[s_cap2_idx]   = (float)self->motor.speed_rad_s;
                self->cap2_va[s_cap2_idx]    = (float)self->motor.v_alpha_prev;
                self->cap2_vb[s_cap2_idx]    = (float)self->motor.v_beta_prev;
                self->cap2_x1[s_cap2_idx]    = (float)self->observer.x1;
                self->cap2_x2[s_cap2_idx]    = (float)self->observer.x2;
                self->cap2_lam[s_cap2_idx]   = (float)self->observer.lambda_est;
                s_cap2_idx++;
            }
            if ((float)self->motor.speed_rad_s < self->cap2_min_spd)
            {
                self->cap2_min_spd = (float)self->motor.speed_rad_s;
            }
            {
                float aiq = (float)self->motor.iq_now;
                if (aiq < 0.0f) { aiq = -aiq; }
                if (aiq > self->cap2_max_iq) { self->cap2_max_iq = aiq; }
            }
        }
    }
    else if (s_prev_ol_stage == 0u && s_cap2_armed != 0u)
    {
        /* 闭环结束（重新开环拖动或停机）：锁存本轮存活拍数 + 整体拷贝影子 */
        self->cap2_ticks = s_cap2_tick;
        self->cap2_valid_n = s_cap2_idx;
        s_cap2_armed = 0u;
        self->cap2_s_switch_count = self->cap2_switch_count;
        self->cap2_s_ticks = self->cap2_ticks;
        self->cap2_s_valid_n = self->cap2_valid_n;
        self->cap2_s_min_spd = self->cap2_min_spd;
        self->cap2_s_max_iq = self->cap2_max_iq;
        memcpy(self->cap2_s_frame, self->cap2_frame, sizeof(self->cap2_frame));
        memcpy(self->cap2_s_obs,   self->cap2_obs,   sizeof(self->cap2_obs));
        memcpy(self->cap2_s_spd,   self->cap2_spd,   sizeof(self->cap2_spd));
        memcpy(self->cap2_s_va,    self->cap2_va,    sizeof(self->cap2_va));
        memcpy(self->cap2_s_vb,    self->cap2_vb,    sizeof(self->cap2_vb));
        memcpy(self->cap2_s_x1,    self->cap2_x1,    sizeof(self->cap2_x1));
        memcpy(self->cap2_s_x2,    self->cap2_x2,    sizeof(self->cap2_x2));
        memcpy(self->cap2_s_lam,   self->cap2_lam,   sizeof(self->cap2_lam));
    }

    s_prev_ol_stage = ol;
}

int drv_motor_calibrate_offset(struct drv_motor *self)
{
    if (self == NULL)
    {
        return -1;
    }

    if (mcl_calibrate_offset(&self->motor) != MCL_OK)
    {
        return -1;
    }

    return 0;
}

/* ==================== R/L 实测 ==================== */

/** 微秒忙等待（基于 HAL_GetTick，1ms 粒度；测量期间控制节拍空闲，忙等待安全） */
static void drv_motor_wait_us(uint32_t us)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t need = (us + 999u) / 1000u;
    if (need == 0u)
    {
        need = 1u;
    }
    while ((uint32_t)(HAL_GetTick() - t0) < need)
    {
        /* busy wait */
    }
}

/** 等 N 个 16kHz 节拍（N×62.5µs），用于 L 脉冲测量等短窗口 */
static void drv_motor_wait_ticks(uint32_t n)
{
    uint32_t t0 = s_jeos_count;
    while ((uint32_t)(s_jeos_count - t0) < n)
    {
        /* busy wait */
    }
}

/** 读 α 相电流（物理 A，已减零漂校准偏置） */
static int drv_motor_read_ia(struct drv_motor *self, float *ia)
{
    mcl_scalar a;
    mcl_scalar b;
    mcl_scalar c;

    if (self == NULL || ia == NULL)
    {
        return -1;
    }

    if (drv_motor_adc_read_phase(self, &a, &b, &c) != MCL_OK)
    {
        return -1;
    }

    a = MCL_SUB(a, self->motor.cfg.current_offset[0]);
    *ia = (float)a;
    return 0;
}

/** 设中性占空比（三相 0.5 → 零电压） */
static void drv_motor_pwm_neutral(struct drv_motor *self)
{
    drv_motor_pwm_set_duty(self, (mcl_scalar)0.5f, (mcl_scalar)0.5f, (mcl_scalar)0.5f);
}

int drv_motor_measure_rl(struct drv_motor *self)
{
    float vbus;
    float ia_sum;
    float ia0;
    float ia1;
    float v_alpha;
    float r_meas;
    float l_meas;
    int i;

    if (self == NULL)
    {
        return -1;
    }
    if (self->motor.state == MCL_STATE_RUN)
    {
        return -1;   /* 控制节拍会抢占 PWM 输出，禁止测量 */
    }

    /* 母线电压（含启动初期 24V 回退） */
    {
        mcl_scalar vb = (mcl_scalar)0;
        mcl_scalar ibus = (mcl_scalar)0;
        (void)drv_motor_adc_read_bus(self, &vb, &ibus);
        vbus = (float)vb;
    }
    if (vbus < 5.0f)
    {
        return -1;
    }

    /*
     * 1) 相电阻：α 轴直流注入。duty 用 mcl SVPWM 约定（0.5=中性）：
     *    da = 0.5 + k/2、db = dc = 0.5 − k/4 → van = k·vbus/2。
     *    k=0.05 → 约 0.6V，R≈0.7Ω 时 ia≈0.85A，稳态 di/dt=0，R = van/ia。
     */
    {
        const float k = 0.05f;
        drv_motor_pwm_set_duty(self, (mcl_scalar)(0.5f + k * 0.5f),
                               (mcl_scalar)(0.5f - k * 0.25f),
                               (mcl_scalar)(0.5f - k * 0.25f));
        drv_motor_wait_us(100000u);   /* 等电流稳定（电气时间常数 L/R ~ms 级） */

        ia_sum = 0.0f;
        for (i = 0; i < 32; i++)
        {
            float ia;
            if (drv_motor_read_ia(self, &ia) != 0)
            {
                drv_motor_pwm_neutral(self);
                return -1;
            }
            ia_sum += ia;
            drv_motor_wait_us(500u);
        }
        drv_motor_pwm_neutral(self);

        ia_sum /= 32.0f;
        if (ia_sum < 0.05f)
        {
            ia_sum = -ia_sum;
        }
        if (ia_sum < 0.05f)
        {
            return -1;   /* 电流太小，测量无效 */
        }

        r_meas = k * vbus * 0.5f / ia_sum;
    }

    /* 2) 消磁：零电压等电流衰减到 0 */
    drv_motor_pwm_neutral(self);
    drv_motor_wait_us(50000u);

    /*
     * 3) 相电感：短电压脉冲测 di/dt。k=0.2 → van ≈ 2.37V，
     *    2 个节拍（125µs）等占空比生效后，测 4 个节拍（250µs）窗口的 Δi。
     *    时序用 16kHz JEOS 节拍计数（与 PWM/ADC 同源，62.5µs/拍精确），
     *    L = (van − R·i_mid)·Δt / Δi（扣电阻压降）。
     */
    {
        const float k = 0.2f;
        float di;
        float dt_s;
        uint32_t c0;
        uint32_t c1;

        drv_motor_pwm_set_duty(self, (mcl_scalar)(0.5f + k * 0.5f),
                               (mcl_scalar)(0.5f - k * 0.25f),
                               (mcl_scalar)(0.5f - k * 0.25f));
        drv_motor_wait_ticks(2u);   /* 125µs：等占空比生效 + 初始电流建立 */

        if (drv_motor_read_ia(self, &ia0) != 0)
        {
            drv_motor_pwm_neutral(self);
            return -1;
        }
        c0 = s_jeos_count;
        drv_motor_wait_ticks(4u);   /* 250µs 窗口 */
        if (drv_motor_read_ia(self, &ia1) != 0)
        {
            drv_motor_pwm_neutral(self);
            return -1;
        }
        c1 = s_jeos_count;
        drv_motor_pwm_neutral(self);

        v_alpha = k * vbus * 0.5f;
        v_alpha -= r_meas * (ia0 + ia1) * 0.5f;   /* 扣电阻压降 */
        di = ia1 - ia0;
        if (di < 0.02f)
        {
            di = -di;
        }
        if (di < 0.02f)
        {
            return -1;   /* 电流未上升，测量无效 */
        }
        dt_s = (float)(c1 - c0) * 62.5e-6f;       /* 每节拍 62.5µs */
        l_meas = v_alpha * dt_s / di;
    }

    /* 合理性校验（超界则判定失败，沿用配置参数） */
    if (r_meas < 0.1f || r_meas > 5.0f)
    {
        return -1;
    }
    if (l_meas < 0.1e-3f || l_meas > 20.0e-3f)
    {
        return -1;
    }

    /* 保存 + 回填观测器/电流环参数 */
    self->r_meas = r_meas;
    self->l_meas = l_meas;

    /* 回填观测器/电流环参数：R、L 用铭牌相值。R = 线 0.95/2 = 0.475Ω（冷态），
       L = 线 Lq 1.6mH/2 = 0.80mH。曾试 R=0.55（运转温度估算）与 R=0.709（脉冲法
       含死区偏高），前者使观测器相位略有超调、后者使 v−R·i 变负磁链反向，均不如
       冷态 0.475 稳，故回填 0.475。 */
    l_meas = 0.8e-3f;
    r_meas = 0.475f;

    self->observer.params.resistance = (mcl_scalar)r_meas;
    self->observer.params.inductance = (mcl_scalar)l_meas;
    self->motor.cfg.phase_resistance = (mcl_scalar)r_meas;
    self->motor.cfg.phase_inductance = (mcl_scalar)l_meas;
    self->motor.foc.mtpa_fw.lq = (mcl_scalar)l_meas;
    self->motor.foc.mtpa_fw.ld = (mcl_scalar)(l_meas - 0.075e-3f);  /* Lq−Ld=0.075mH 凸极差 */

    return 0;
}

int drv_motor_get_telemetry(struct drv_motor *self, mcl_telemetry *out)
{
    if ((self == NULL) || (out == NULL))
    {
        return -1;
    }

    if (mcl_get_telemetry(&self->motor, out) != MCL_OK)
    {
        return -1;
    }

    return 0;
}
