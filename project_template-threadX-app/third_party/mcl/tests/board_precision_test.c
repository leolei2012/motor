/* Actual board defaults across all precisions. Board-scale closed-loop regression. Average PWM, floating star point,
 * independent PMSM mechanical dynamics and quadratic fan load. */
#include "mcl.h"
#include "mcl_observer_smo.h"
#include "drv_motor_config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "mcl_calibration.h"

static int offset_samples(void *ctx, mcl_scalar *a, mcl_scalar *b, mcl_scalar *c)
{
    (void)ctx;
    *a = MCL_CONFIG_VALUE(0.05f, DRV_MOTOR_I_BASE);
    *b = MCL_CONFIG_VALUE(-0.07f, DRV_MOTOR_I_BASE);
    *c = MCL_CONFIG_VALUE(0.02f, DRV_MOTOR_I_BASE);
    return MCL_OK;
}

static void units_and_config(void)
{
    mcl motor = {0};
    mcl_config cfg;
    mcl_observer_smo_params op;
    mcl_scalar alpha, beta;
    mcl_scalar offset[3];
    mcl_hal_ops hal = {0};
    hal.adc_read_phase = offset_samples;
    assert(mcl_cal_current_offset(&hal, 0, 4096, offset) == MCL_OK);
    assert(fabsf(mcl_to_physical(offset[0], DRV_MOTOR_I_BASE) - 0.05f) < 0.001f);
    assert(fabsf(mcl_to_physical(offset[1], DRV_MOTOR_I_BASE) + 0.07f) < 0.001f);
    assert(fabsf(mcl_to_physical(offset[2], DRV_MOTOR_I_BASE) - 0.02f) < 0.001f);
    drv_motor_config_default(&cfg, &op);
    {
        mcl_scalar sn, cs;
        mcl_math_sincos(MCL_CONFIG_VALUE(5.49778714f, DRV_MOTOR_ANGLE_BASE), &sn, &cs);
        assert(fabsf(MCL_TO_FLOAT(sn) + 0.70710678f) < 0.002f);
        assert(fabsf(MCL_TO_FLOAT(cs) - 0.70710678f) < 0.002f);
    }
    assert(mcl_config_validate(&cfg) == MCL_OK);
    assert(cfg.openloop_time_ramp == 1.5f);
    assert(fabsf(mcl_to_physical(cfg.phase_resistance, DRV_MOTOR_R_BASE) - 0.475f) < 0.0002f);
    assert(fabsf(mcl_to_physical(mcl_from_physical(-2.5f, DRV_MOTOR_I_BASE), DRV_MOTOR_I_BASE) + 2.5f) < 0.002f);
    assert(fabsf(MCL_TO_FLOAT(mcl_mul_div(MCL_FROM_FLOAT(0.8f), MCL_FROM_FLOAT(0.1f), MCL_FROM_FLOAT(0.2f))) - 0.4f) < 0.001f);
    mcl_transform_clarke(MCL_FROM_FLOAT(-0.8f), MCL_FROM_FLOAT(0.8f), 0, &alpha, &beta);
    assert(fabsf(MCL_TO_FLOAT(beta) - 0.8f / 1.7320508f) < 0.001f);
    motor.cfg = cfg;
    assert(mcl_set_torque(&motor, MCL_CONFIG_VALUE(0.05f, DRV_MOTOR_FLUX_BASE * DRV_MOTOR_I_BASE)) == MCL_OK);
    assert(fabsf(mcl_to_physical(motor.iq_ref, DRV_MOTOR_I_BASE) - 0.05f / (1.5f * 5.0f * 0.00717f)) < 0.003f);
    cfg.openloop_time_ramp = -1.0f;
    assert(mcl_config_validate(&cfg) == MCL_ERR_PARAM);
}

typedef struct
{
    float ia, ib, theta, omega, va, vb, inertia, fan;
    float max_current;
    int bad_pwm;
    int locked;
    float noise;
    unsigned sample;
} plant;

static float wrap(float x)
{
    while (x > MCL_PI) x -= MCL_TWO_PI;
    while (x < -MCL_PI) x += MCL_TWO_PI;
    return x;
}

static void pwm(void *ctx, mcl_scalar qa, mcl_scalar qb, mcl_scalar qc)
{
    plant *p = ctx;
    float a=MCL_TO_FLOAT(qa), b=MCL_TO_FLOAT(qb), c=MCL_TO_FLOAT(qc);
    float mean = (a + b + c) / 3.0f;
    if (!isfinite(a+b+c) || a < 0 || b < 0 || c < 0 || a > 0.95001f || b > 0.95001f || c > 0.95001f)
        p->bad_pwm = 1;
    p->va = 24.0f * (a - mean);
    p->vb = 24.0f * (b - c) / 1.73205080757f;
}

static int adc(void *ctx, mcl_scalar *a, mcl_scalar *b, mcl_scalar *c)
{
    plant *p = ctx;
    const float dt = 1.0f / (16000.0f * 8.0f);
    int k;
    for (k = 0; k < 8; ++k)
    {
        float sn = sinf(p->theta), cs = cosf(p->theta);
        float iq = -p->ia * sn + p->ib * cs;
        float wm = p->omega / 5.0f;
        float load = p->fan * wm * fabsf(wm) + 0.00001f * wm;
        p->ia += (p->va - 0.475f * p->ia + p->omega * 0.00717f * sn) * dt / 0.0008f;
        p->ib += (p->vb - 0.475f * p->ib - p->omega * 0.00717f * cs) * dt / 0.0008f;
        p->omega += 5.0f * (1.5f * 5.0f * 0.00717f * iq - load) * dt / p->inertia;
        if (p->locked) p->omega = 0;
        p->theta = wrap(p->theta + p->omega * dt);
    }
    float a_phys = p->ia;
    float b_phys = -0.5f * p->ia + 0.86602540378f * p->ib;
    float c_phys = -0.5f * p->ia - 0.86602540378f * p->ib;
    /* Repeatable current measurement error, independent of motor dynamics. */
    ++p->sample;
    a_phys += p->noise * sinf((float)p->sample * 1.73f);
    b_phys += p->noise * cosf((float)p->sample * 2.31f);
    *a=mcl_from_physical(a_phys,DRV_MOTOR_I_BASE);
    *b=mcl_from_physical(b_phys,DRV_MOTOR_I_BASE);
    *c=mcl_from_physical(c_phys,DRV_MOTOR_I_BASE);
    if (hypotf(p->ia,p->ib) > p->max_current) p->max_current = hypotf(p->ia,p->ib);
    return MCL_OK;
}

static int bus(void *ctx, mcl_scalar *v, mcl_scalar *i)
{
    (void)ctx; *v = MCL_CONFIG_VALUE(24.0f,DRV_MOTOR_V_BASE); *i = 0.0f; return MCL_OK;
}

static int run(float target, float angle, float inertia, int loaded, float noise)
{
    mcl motor;
    mcl_config cfg;
    mcl_observer_smo obs;
    mcl_observer_smo_params op;
    mcl_hal_ops hal = {0};
    plant p = {0};
    float sum = 0, sumerr = 0, jump = 0, prev_phase = 0;
    int n, transitions = 0, prev_stage = 0;
    p.theta = angle; p.inertia = inertia;
    p.noise = noise;
    p.fan = loaded ? (1.5f * 5 * 0.00717f * 1.7f) / (83.775804f * 83.775804f) : 0;
    hal.pwm_set_duty = pwm; hal.adc_read_phase = adc; hal.adc_read_bus = bus;
    drv_motor_config_default(&cfg,&op);
    memset(&motor, 0xA5, sizeof(motor)); /* init must initialize transition fields */
    if (mcl_init(&motor,&cfg,&hal,&p,&mcl_observer_smo_ops,&obs,&op) != MCL_OK) return 1;
    mcl_set_speed(&motor,mcl_from_physical(target,DRV_MOTOR_RPM_BASE)); mcl_start(&motor);
    for (n = 0; n < 160000; ++n)
    {
        mcl_control_tick(&motor);
        if (!isfinite(p.omega) || motor.state != MCL_STATE_RUN) break;
        if (prev_stage == 2 && motor.ol_stage == 0)
        {
            float step = fabsf(wrap(mcl_to_physical(motor.phase_rad,DRV_MOTOR_ANGLE_BASE)-prev_phase));
            if (step > jump) jump = step;
            ++transitions;
        }
        prev_stage = motor.ol_stage; prev_phase = mcl_to_physical(motor.phase_rad,DRV_MOTOR_ANGLE_BASE);
        if (n >= 144000)
        {
            sum += p.omega / 5.0f * 9.5492966f;
            sumerr += fabsf(wrap(mcl_to_physical(motor.phase_rad,DRV_MOTOR_ANGLE_BASE)-p.theta));
        }
    }
    printf("target=%5.0f angle=%.1f J=%.5f load=%d noise=%.3f n=%d rpm=%.1f phase=%.2fdeg switches=%d jump=%.2fdeg peakI=%.2f fault=%d\n",
        target,angle,inertia,loaded,noise,n,sum/16000,sumerr/16000*57.29578f,transitions,jump*57.29578f,p.max_current,motor.fault);
    return n != 160000 || transitions != 1 || fabsf(sum/16000-target) > fabsf(target)*0.01f ||
        sumerr/16000 > 0.175f || jump > 0.10f || p.bad_pwm;
}

static int locked_rotor(void)
{
    mcl motor = {0};
    mcl_config cfg;
    mcl_observer_smo obs;
    mcl_observer_smo_params op;
    mcl_hal_ops hal = {0};
    plant p = {0};
    int n;
    p.inertia = 0.0001f; p.locked = 1;
    hal.pwm_set_duty = pwm; hal.adc_read_phase = adc; hal.adc_read_bus = bus;
    drv_motor_config_default(&cfg,&op);
    mcl_init(&motor,&cfg,&hal,&p,&mcl_observer_smo_ops,&obs,&op);
    mcl_set_speed(&motor,MCL_CONFIG_VALUE(800,DRV_MOTOR_RPM_BASE)); mcl_start(&motor);
    for (n=0; n<40000; ++n) mcl_control_tick(&motor);
    printf("Locked rotor: fault=%d state=%d stage=%u after %.3fs\n",
           motor.fault, motor.state, motor.ol_stage, 40000/16000.0f);
    /* 转子锁住时观测器不合格：继续开环拖，不报堵转、不关断。 */
    if (motor.fault != MCL_FAULT_NONE || motor.state != MCL_STATE_RUN || motor.ol_stage == 0u) return 1;
    mcl_stop(&motor);
    return 0;
}

int main(void)
{
    int failures = 0;
    units_and_config();
    failures += run(800, 0.0f, 0.0001f, 1, 0);
    failures += run(800, 1.0f, 0.0001f, 1, 0);
    failures += run(800, 2.5f, 0.0001f, 1, 0);
    failures += run(800, 0.0f, 0.0002f, 1, 0);
    failures += run(300, 0.0f, 0.0001f, 0, 0);
    failures += run(-800, 1.0f, 0.0001f, 1, 0);
    failures += run(800, 0.0f, 0.0001f, 1, 0.02f);
    failures += locked_rotor();
    printf("Startup regression: %d failures\n",failures);
    return failures ? 1 : 0;
}
