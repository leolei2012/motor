/* Board-scale closed-loop regression. Average PWM, floating star point,
 * independent PMSM mechanical dynamics and quadratic fan load. */
#include "mcl.h"
#include "mcl_observer_smo.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

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

static void pwm(void *ctx, mcl_scalar a, mcl_scalar b, mcl_scalar c)
{
    plant *p = ctx;
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
    *a = p->ia;
    *b = -0.5f * p->ia + 0.86602540378f * p->ib;
    *c = -0.5f * p->ia - 0.86602540378f * p->ib;
    /* Repeatable current measurement error, independent of motor dynamics. */
    ++p->sample;
    *a += p->noise * sinf((float)p->sample * 1.73f);
    *b += p->noise * cosf((float)p->sample * 2.31f);
    if (hypotf(p->ia,p->ib) > p->max_current) p->max_current = hypotf(p->ia,p->ib);
    return MCL_OK;
}

static int bus(void *ctx, mcl_scalar *v, mcl_scalar *i)
{
    (void)ctx; *v = 24.0f; *i = 0.0f; return MCL_OK;
}

static void setup(mcl_config *cfg, float target)
{
    mcl_config_default(cfg);
    cfg->pole_pairs = 5;
    cfg->phase_resistance = 0.475f; cfg->phase_inductance = 0.0008f;
    cfg->ld_lq_diff = 0.000075f; cfg->bemf_const = 0.00717f;
    cfg->rated_current = 4.0f;
    cfg->pwm_freq_hz = cfg->current_loop_freq_hz = 16000;
    cfg->speed_loop_divider = 16;
    cfg->current_pid.kp = 0.04f; cfg->current_pid.ki = 4.0f;
    cfg->speed_pid.kp = 0.005f; cfg->speed_pid.ki = 0.02f; cfg->speed_pid.kd = 0;
    cfg->speed_pid.out_min = cfg->speed_pid.i_min = -3.0f;
    cfg->speed_pid.out_max = cfg->speed_pid.i_max = 3.0f;
    cfg->pll_kp = 200.0f; cfg->pll_ki = 10000.0f;
    cfg->openloop_rpm = fabsf(target);
    cfg->openloop_drag_q = 3.0f; cfg->openloop_time_lock = 0;
    cfg->openloop_time_ramp = 1.5f; cfg->openloop_time = 0.05f;
    cfg->limits.enabled = MCL_PROTECT_OVERCURRENT | MCL_PROTECT_OVERVOLTAGE | MCL_PROTECT_UNDERVOLTAGE;
    cfg->limits.overcurrent = 4.0f; cfg->limits.overvoltage = 30; cfg->limits.undervoltage = 10;
    cfg->fault_stop_time = 0;
}

static int run(float target, float angle, float inertia, int loaded, float noise)
{
    mcl motor;
    mcl_config cfg;
    mcl_observer_smo obs;
    mcl_observer_smo_params op = {0.475f,0.0008f,0.00717f,10.0f,6.28f,0.5f};
    mcl_hal_ops hal = {0};
    plant p = {0};
    float sum = 0, sumerr = 0, jump = 0, prev_phase = 0;
    int n, transitions = 0, prev_stage = 0;
    p.theta = angle; p.inertia = inertia;
    p.noise = noise;
    p.fan = loaded ? (1.5f * 5 * 0.00717f * 1.7f) / (83.775804f * 83.775804f) : 0;
    hal.pwm_set_duty = pwm; hal.adc_read_phase = adc; hal.adc_read_bus = bus;
    setup(&cfg, target);
    memset(&motor, 0xA5, sizeof(motor)); /* init must initialize transition fields */
    if (mcl_init(&motor,&cfg,&hal,&p,&mcl_observer_smo_ops,&obs,&op) != MCL_OK) return 1;
    mcl_set_speed(&motor,target); mcl_start(&motor);
    for (n = 0; n < 160000; ++n)
    {
        mcl_control_tick(&motor);
        if (!isfinite(p.omega) || motor.state != MCL_STATE_RUN) break;
        if (prev_stage == 2 && motor.ol_stage == 0)
        {
            float step = fabsf(wrap(motor.phase_rad-prev_phase));
            if (step > jump) jump = step;
            ++transitions;
        }
        prev_stage = motor.ol_stage; prev_phase = motor.phase_rad;
        if (n >= 144000)
        {
            sum += p.omega / 5.0f * 9.5492966f;
            sumerr += fabsf(wrap(motor.phase_rad-p.theta));
        }
    }
    printf("target=%5.0f angle=%.1f J=%.5f load=%d noise=%.3f n=%d rpm=%.1f phase=%.2fdeg switches=%d jump=%.2fdeg peakI=%.2f fault=%d\n",
        target,angle,inertia,loaded,noise,n,sum/16000,sumerr/16000*57.29578f,transitions,jump*57.29578f,p.max_current,motor.fault);
    return n != 160000 || transitions != 1 || fabsf(sum/16000-target) > fabsf(target)*0.05f ||
        sumerr/16000 > 0.175f || jump > 0.10f || p.bad_pwm;
}

static int locked_rotor(void)
{
    mcl motor = {0};
    mcl_config cfg;
    mcl_observer_smo obs;
    mcl_observer_smo_params op = {0.475f,0.0008f,0.00717f,10.0f,6.28f,0.5f};
    mcl_hal_ops hal = {0};
    plant p = {0};
    int n;
    p.inertia = 0.0001f; p.locked = 1;
    hal.pwm_set_duty = pwm; hal.adc_read_phase = adc; hal.adc_read_bus = bus;
    setup(&cfg, 800);
    mcl_init(&motor,&cfg,&hal,&p,&mcl_observer_smo_ops,&obs,&op);
    mcl_set_speed(&motor,800); mcl_start(&motor);
    for (n=0; n<40000 && motor.state == MCL_STATE_RUN; ++n) mcl_control_tick(&motor);
    printf("Locked rotor: fault=%d after %.3fs, applied V=(%.4f,%.4f)\n",motor.fault,n/16000.0f,p.va,p.vb);
    if (motor.fault != MCL_FAULT_STALL || p.va != 0 || p.vb != 0) return 1;
    for (n=0;n<16000;++n) mcl_control_tick(&motor);
    if (motor.state != MCL_STATE_FAULT || mcl_start(&motor) != MCL_ERR_STATE) return 1;
    /* Clearing a latched fault and restarting must not reuse a pending blend. */
    mcl_clear_fault(&motor);
    motor.switch_blend_timer = 0.3f; motor.switch_phase_offset = 1.0f;
    if (mcl_start(&motor) != MCL_OK || motor.switch_blend_timer != 0 ||
        motor.switch_phase_offset != 0 || obs.seed_omega != 0 || obs.filter_step != 0) return 1;
    mcl_stop(&motor);
    return 0;
}

int main(void)
{
    int failures = 0;
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
