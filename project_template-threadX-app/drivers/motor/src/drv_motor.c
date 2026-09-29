#include "drv_motor.h"
#include "drv_motor_units.h"
#include "drv_motor_config.h"

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
    *ia = mcl_from_physical((((float)CURRENT_ADC_MID - (float)ia_raw) * CURRENT_ADC_SCALE_A), DRV_MOTOR_I_BASE);
    *ib = mcl_from_physical((((float)CURRENT_ADC_MID - (float)ib_raw) * CURRENT_ADC_SCALE_A), DRV_MOTOR_I_BASE);
    *ic = mcl_from_physical((((float)CURRENT_ADC_MID - (float)ic_raw) * CURRENT_ADC_SCALE_A), DRV_MOTOR_I_BASE);
#else
    *ia = mcl_from_physical((((float)ia_raw - CURRENT_ADC_MID) * CURRENT_ADC_SCALE_A), DRV_MOTOR_I_BASE);
    *ib = mcl_from_physical((((float)ib_raw - CURRENT_ADC_MID) * CURRENT_ADC_SCALE_A), DRV_MOTOR_I_BASE);
    *ic = mcl_from_physical((((float)ic_raw - CURRENT_ADC_MID) * CURRENT_ADC_SCALE_A), DRV_MOTOR_I_BASE);
#endif

    /* 调试观测：保存减零漂后的三相电流（与 FOC 实际使用的一致） */
    if (self != NULL)
    {
        mcl_scalar offsets[3];
        (void)mcl_get_current_offsets(&self->motor, offsets);
        self->ia_now = mcl_to_physical(MCL_SUB(*ia, offsets[0]), DRV_MOTOR_I_BASE);
        self->ib_now = mcl_to_physical(MCL_SUB(*ib, offsets[1]), DRV_MOTOR_I_BASE);
        self->ic_now = mcl_to_physical(MCL_SUB(*ic, offsets[2]), DRV_MOTOR_I_BASE);
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
        *vbus = mcl_from_physical((float)g_drv.ain_sensor->bus_voltage.voltage_mv / 1000.0f, DRV_MOTOR_V_BASE);
    }
    else
    {
        *vbus = MCL_CONFIG_VALUE(24.0f, DRV_MOTOR_V_BASE);
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
    uint16_t cmp_a;
    uint16_t cmp_b;
    uint16_t cmp_c;
    if (!(da >= 0.0f)) { da = 0.0f; }
    if (!(db >= 0.0f)) { db = 0.0f; }
    if (!(dc >= 0.0f)) { dc = 0.0f; }
    if (da > MCL_FROM_FLOAT(1.0f)) { da = MCL_FROM_FLOAT(1.0f); }
    if (db > MCL_FROM_FLOAT(1.0f)) { db = MCL_FROM_FLOAT(1.0f); }
    if (dc > MCL_FROM_FLOAT(1.0f)) { dc = MCL_FROM_FLOAT(1.0f); }
    cmp_a = (uint16_t)(MCL_TO_FLOAT(da) * (float)PWM_HALF_PERIOD);
    cmp_b = (uint16_t)(MCL_TO_FLOAT(db) * (float)PWM_HALF_PERIOD);
    cmp_c = (uint16_t)(MCL_TO_FLOAT(dc) * (float)PWM_HALF_PERIOD);
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
    mcl_observer_smo_params op;
    drv_motor_config_default(&cfg, &op);

    if (mcl_init(&self->motor, &cfg, &s_mcl_hal, self,
                 &mcl_observer_smo_ops, &self->observer, &op) != MCL_OK)
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

    if (mcl_set_current(&self->motor, mcl_from_physical(iq_ref, DRV_MOTOR_I_BASE)) != MCL_OK)
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

    if (mcl_set_speed(&self->motor, mcl_from_physical(speed_rpm, DRV_MOTOR_RPM_BASE)) != MCL_OK)
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

    if (mcl_set_openloop_vf(&self->motor, mcl_from_physical(voltage, 1.0f), mcl_from_physical(speed_rpm, DRV_MOTOR_RPM_BASE)) != MCL_OK)
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

    if (mcl_set_openloop_if(&self->motor, mcl_from_physical(current, DRV_MOTOR_I_BASE), mcl_from_physical(speed_rpm, DRV_MOTOR_RPM_BASE)) != MCL_OK)
    {
        return -1;
    }

    return 0;
}

/** 上一拍开环阶段（切换捕获用） */
static uint8_t s_prev_ol_stage = 0u;

/** 切换后逐拍采集：对数间隔偏移（1,2,4,...,8192 拍 @16kHz = 62.5µs~512ms）。
    覆盖整段闭环存活期（死亡点 ~150ms~1s 不定），抓撒手瞬态 + 转子摆动 + 掉速全过程。 */
static const uint32_t s_cap2_off[14] = {1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u, 256u, 512u, 1024u, 2048u, 4096u, 8192u};
static uint8_t  s_cap2_idx   = 0u;   /**< 下一个待采集偏移索引 */
static uint8_t  s_cap2_armed = 0u;   /**< 1=本轮闭环采集进行中 */
static uint32_t s_cap2_tick  = 0u;   /**< 本轮回合内拍数 */

void drv_motor_control_isr(struct drv_motor *self)
{
    uint8_t ol;
    mcl_observer_smo_diagnostics obs;

    if (self == NULL)
    {
        return;
    }

    s_jeos_count++;
    mcl_control_tick(&self->motor);
    (void)mcl_observer_smo_get_diagnostics(&self->observer, &obs);

    /* 切换瞬间事件捕获：拖动(2)→闭环(0) 的帧角/观测器角/速度，
       定位切闭环崩溃的相位跳变（一次性，cap_valid 置位后不再覆盖） */
    ol = (uint8_t)mcl_get_ol_stage(&self->motor);
    if (ol == 2u)
    {
        self->cap_pre_est = mcl_to_physical(mcl_get_phase_rad(&self->motor), DRV_MOTOR_ANGLE_BASE);
        self->cap_pre_lam = mcl_to_physical(obs.phase, DRV_MOTOR_ANGLE_BASE);
        self->cap_in_prev2 = self->cap_in_prev1;
        self->cap_in_prev1 = mcl_to_physical(mcl_get_pll_last_phase(&self->motor), DRV_MOTOR_ANGLE_BASE);   /* = 本拍 seed 前观测器输出（PLL 输入） */
    }
    else if (ol == 0u && s_prev_ol_stage == 2u && self->cap_valid == 0u)
    {
        self->cap_post_est = mcl_to_physical(mcl_get_phase_rad(&self->motor), DRV_MOTOR_ANGLE_BASE);
        self->cap_post_lam = mcl_to_physical(obs.phase, DRV_MOTOR_ANGLE_BASE);
        self->cap_post_spd = mcl_to_physical(mcl_get_speed_rad_s(&self->motor), DRV_MOTOR_W_BASE);
        self->cap_pll_last = mcl_to_physical(mcl_get_pll_last_phase(&self->motor), DRV_MOTOR_ANGLE_BASE);
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
                self->cap2_frame[s_cap2_idx] = mcl_to_physical(mcl_get_phase_rad(&self->motor), DRV_MOTOR_ANGLE_BASE);
                self->cap2_obs[s_cap2_idx]   = (float)self->ia_now;                  /* 复用：i_α A（实测） */
                self->cap2_spd[s_cap2_idx]   = (float)((self->ia_now + 2.0f * self->ib_now) * 0.57735027f); /* 复用：i_β A（实测） */
                self->cap2_va[s_cap2_idx]    = mcl_to_physical(obs.i_alpha_hat, DRV_MOTOR_I_BASE);   /* 复用：i_hat_α A（SMO 估计） */
                self->cap2_vb[s_cap2_idx]    = mcl_to_physical(obs.i_beta_hat, DRV_MOTOR_I_BASE);    /* 复用：i_hat_β A */
                self->cap2_x1[s_cap2_idx]    = mcl_to_physical(obs.e_alpha_final, DRV_MOTOR_V_BASE); /* SMO 反电动势 α */
                self->cap2_x2[s_cap2_idx]    = mcl_to_physical(obs.e_beta_final, DRV_MOTOR_V_BASE); /* SMO 反电动势 β */
                self->cap2_lam[s_cap2_idx]   = mcl_to_physical(obs.z_alpha, DRV_MOTOR_V_BASE);  /* 复用：SMO 滑模输出 z_α */
                s_cap2_idx++;
            }
            if (mcl_to_physical(mcl_get_speed_rad_s(&self->motor), DRV_MOTOR_W_BASE) < self->cap2_min_spd)
            {
                self->cap2_min_spd = mcl_to_physical(mcl_get_speed_rad_s(&self->motor), DRV_MOTOR_W_BASE);
            }
            {
                float aiq = mcl_to_physical(mcl_get_iq_now(&self->motor), DRV_MOTOR_I_BASE);
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

    mcl_scalar offsets[3];
    (void)mcl_get_current_offsets(&self->motor, offsets);
    a = MCL_SUB(a, offsets[0]);
    *ia = mcl_to_physical(a, DRV_MOTOR_I_BASE);
    return 0;
}

/** 设中性占空比（三相 0.5 → 零电压） */
static void drv_motor_pwm_neutral(struct drv_motor *self)
{
    drv_motor_pwm_set_duty(self, MCL_FROM_FLOAT(0.5f), MCL_FROM_FLOAT(0.5f), MCL_FROM_FLOAT(0.5f));
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
    mcl_state state;
    if (mcl_get_state(&self->motor, &state) != MCL_OK || state != MCL_STATE_IDLE)
    {
        return -1;   /* 控制节拍会抢占 PWM 输出，禁止测量 */
    }

    /* 母线电压（含启动初期 24V 回退） */
    {
        mcl_scalar vb = (mcl_scalar)0;
        mcl_scalar ibus = (mcl_scalar)0;
        (void)drv_motor_adc_read_bus(self, &vb, &ibus);
        vbus = mcl_to_physical(vb, DRV_MOTOR_V_BASE);
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
        drv_motor_pwm_set_duty(self, MCL_FROM_FLOAT(0.5f + k * 0.5f),
                               MCL_FROM_FLOAT(0.5f - k * 0.25f),
                               MCL_FROM_FLOAT(0.5f - k * 0.25f));
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

        drv_motor_pwm_set_duty(self, MCL_FROM_FLOAT(0.5f + k * 0.5f),
                               MCL_FROM_FLOAT(0.5f - k * 0.25f),
                               MCL_FROM_FLOAT(0.5f - k * 0.25f));
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

    mcl_motor_parameters params;
    if (mcl_get_motor_parameters(&self->motor, &params) != MCL_OK) { return -1; }
    params.phase_resistance = mcl_from_physical(r_meas, DRV_MOTOR_R_BASE);
    params.phase_inductance = mcl_from_physical(l_meas, DRV_MOTOR_L_BASE);
    if (mcl_set_motor_parameters(&self->motor, &params) != MCL_OK) { return -1; }

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

int drv_motor_get_diagnostics(const struct drv_motor *self, mcl_diagnostics *motor,
                              mcl_observer_smo_diagnostics *observer)
{
    uint32_t irq_mask;
    int result;
    if (self == NULL || motor == NULL || observer == NULL) { return -1; }
    irq_mask = __get_PRIMASK();
    __disable_irq();
    result = mcl_get_diagnostics(&self->motor, motor);
    if (result == MCL_OK)
    {
        result = mcl_observer_smo_get_diagnostics(&self->observer, observer);
    }
    __set_PRIMASK(irq_mask);
    return result == MCL_OK ? 0 : -1;
}
