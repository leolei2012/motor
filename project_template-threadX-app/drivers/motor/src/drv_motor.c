#include "drv_motor.h"

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
 *   I = (raw - 32768) / 65536 × 3.3 / (0.02 × 6)
 *     = (raw - 32768) × 0.0004196
 */
#define CURRENT_ADC_SCALE_A     0.0004196f   /**< 3.3/(0.02×6)/65536，左对齐 A/count */

/* 量程中点（12bit 左对齐 16bit 量程的一半），作为 raw→A 的参考零点 */
#define CURRENT_ADC_MID         32768.0f

/* ==================== mcl_hal_ops 回调 ==================== */

/** 读取三相电流（ADC 注入组，左对齐 12bit → 安培，含偏置，零漂由 mcl 校准） */
static int drv_motor_adc_read_phase(void *ctx, mcl_scalar *ia, mcl_scalar *ib, mcl_scalar *ic)
{
    (void)ctx;

    uint16_t ia_raw = hal_adc1_inj_read_jdr1();  /* U 相 (ADC1 rank1) */
    uint16_t ib_raw = hal_adc1_inj_read_jdr2();  /* V 相 (ADC1 rank2) */
    uint16_t ic_raw = hal_adc2_inj_read_jdr2();  /* W 相 (ADC2 rank2) */

    *ia = (mcl_scalar)(((float)ia_raw - CURRENT_ADC_MID) * CURRENT_ADC_SCALE_A);
    *ib = (mcl_scalar)(((float)ib_raw - CURRENT_ADC_MID) * CURRENT_ADC_SCALE_A);
    *ic = (mcl_scalar)(((float)ic_raw - CURRENT_ADC_MID) * CURRENT_ADC_SCALE_A);

    return MCL_OK;
}

/** 读取母线电压/电流（母线电压来自 drv_ain_sensor） */
static int drv_motor_adc_read_bus(void *ctx, mcl_scalar *vbus, mcl_scalar *ibus)
{
    (void)ctx;

    /*
     * 母线电压由 drv_ain_sensor 采样并换算成 mV，这里转成 V。
     * 母线电流无传感器，暂填 0。
     */
    if (g_drv.ain_sensor != NULL)
    {
        *vbus = (mcl_scalar)((float)g_drv.ain_sensor->bus_voltage.voltage_mv / 1000.0f);
    }
    else
    {
        *vbus = 0.0f;
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

    hal_tim1_set_duty_abc(cmp_a, cmp_b, cmp_c);
}

/** 微秒时间基准 */
static uint32_t drv_motor_micros(void *ctx)
{
    (void)ctx;
    return 0u;  /* 暂未用 */
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

    /* —— 电机实体参数（UserData_Motor.h，物理量 1:1 映射）—— */
    cfg.pole_pairs          = 10u;                /* Poles=10 对极 */
    cfg.phase_resistance    = 0.95f;              /* Rs=0.95Ω */
    cfg.phase_inductance    = 1.6e-3f;            /* Lq=1.6mH */
    cfg.ld_lq_diff          = 0.15e-3f;           /* Lq-Ld = 1.6mH-1.45mH = 0.15mH (IPMSM) */
    cfg.bemf_const          = 3.586e-3f;          /* 磁链 λ=3.586mVs/rad */
    cfg.rated_current       = 4.0f;               /* Qcur_MAX=4A */
    cfg.rated_speed_rpm     = 3000.0f;            /* 参考值，默认保留 */
    cfg.bus_voltage         = 24.0f;              /* VBUS=24V */

    /* —— 运行频率 —— */
    cfg.pwm_freq_hz         = 16000u;             /* 与 hal_tim1 PWM 一致 */
    cfg.current_loop_freq_hz = 16000u;            /* 电流环 = PWM 频率 */
    cfg.speed_loop_divider  = 16u;                /* 速度环 1kHz */

    /* —— 反馈：无感 —— */
    cfg.feedback.type = MCL_FEEDBACK_NONE;

    /* —— 电流环 PID（老工程 Kp=0.1V/A, Ki=48V/(A·s)，标幺化 ÷24V 母线。
          mcl 输出为归一化电压 [-1,1]，与老工程物理电压量纲不同，需实测整定）—— */
    cfg.current_pid.kp = 0.1f / 24.0f;            /* ≈ 0.00417 */
    cfg.current_pid.ki = 48.0f / 24.0f;           /* ≈ 2.0 */
    cfg.current_pid.kd = 0.0f;
    cfg.current_pid.out_min = -1.0f;
    cfg.current_pid.out_max = 1.0f;
    cfg.current_pid.i_min   = -1.0f;
    cfg.current_pid.i_max   = 1.0f;

    /* —— 速度环 PID（老工程 Kp=0.5A/(rad/s), Ki=2.0A/(rad/s·s), 输出限 ±1.5A）—— */
    cfg.speed_pid.kp = 0.5f;
    cfg.speed_pid.ki = 2.0f;
    cfg.speed_pid.kd = 0.0f;
    cfg.speed_pid.out_min = -1.5f;
    cfg.speed_pid.out_max = 1.5f;
    cfg.speed_pid.i_min   = -1.5f;
    cfg.speed_pid.i_max   = 1.5f;

    /* —— PLL（老工程 PLL 参数，用于无感速度跟踪）—— */
    cfg.pll_kp = 55.0f;
    cfg.pll_ki = 11448.0f;

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

    /* 磁链观测器参数（用电机实体参数） */
    mcl_observer_flux_params op;
    op.lambda     = cfg.bemf_const;
    op.resistance = cfg.phase_resistance;
    op.inductance = cfg.phase_inductance;
    op.gain       = 100.0f;

    if (mcl_init(&self->motor, &cfg, &s_mcl_hal, self,
                 &mcl_observer_flux_ops, &self->observer, &op) != MCL_OK)
    {
        return -1;
    }

    mcl_set_mode(&self->motor, MCL_MODE_FOC_SENSORLESS);

    /*
     * 电流零漂动态校准：电机停转、PWM 无输出时采样 256 次平均，
     * 把器件偏置（1.25V 等）与运放漂移记为 current_offset，
     * 之后 mcl 每次控制节拍自动减去。
     * 注意：必须在 mcl_init 之后、且 ADC 已就绪（hal_init 已完成）时调用。
     */
    if (mcl_calibrate_offset(&self->motor) != MCL_OK)
    {
        return -1;
    }

    return 0;
}

/* ==================== 封装面门 API ==================== */

int drv_motor_start(struct drv_motor *self)
{
    if (self == NULL)
    {
        return -1;
    }

    if (mcl_start(&self->motor) != MCL_OK)
    {
        return -1;
    }

    /*
     * 启动硬件时序（参考 ST MCSDK）：
     *   1. 使能驱动器（M1_EN_DRIVER）
     *   2. 启动 TIM1 计数器
     *   3. 使能 PWM 输出（MOE）
     */
    drv_output_set(g_drv.output, DRV_OUTPUT_EN_DRIVER, DRV_OUTPUT_ON);
    hal_tim1_start();
    hal_tim1_pwm_enable();

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

void drv_motor_control_isr(struct drv_motor *self)
{
    if (self == NULL)
    {
        return;
    }

    mcl_control_tick(&self->motor);
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
