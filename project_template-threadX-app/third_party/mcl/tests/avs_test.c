#include "mcl.h"
#include "mcl_avs.h"
#include <assert.h>
#include <stdio.h>

typedef struct { float voltage; float speed; } fixture;
static int phase(void *c, float *a, float *b, float *d)
{ (void)c; *a = *b = *d = 0; return MCL_OK; }
static int bus(void *c, float *v, float *i)
{ *v = ((fixture *)c)->voltage; *i = 0; return MCL_OK; }
static int speed(void *c, float *s)
{ *s = ((fixture *)c)->speed; return MCL_OK; }
static void pwm(void *c, float a, float b, float d)
{ (void)c; assert(isfinite(a+b+d)); }

int main(void)
{
    mcl_config cfg;
    mcl motor = {0};
    fixture f = {24, 500};
    mcl_hal_ops hal = {0};
    mcl_pid pid;
    int i;
    float value;
    mcl_config_default(&cfg);
    assert(!cfg.avs_enabled);
    assert(cfg.speed_aw_time == 0);
    cfg.avs_enabled = true;
    cfg.speed_aw_time = 0.02f;
    cfg.avs_start_voltage = 28;
    cfg.avs_stop_voltage = 30;
    cfg.limits.overvoltage = 36;
    cfg.limits.enabled = MCL_PROTECT_OVERVOLTAGE;
    cfg.fault_stop_time = 0;
    cfg.speed_pid.kp = 0.005f;
    cfg.speed_pid.ki = 0.02f;
    cfg.speed_pid.out_min = cfg.speed_pid.i_min = -3;
    cfg.speed_pid.out_max = cfg.speed_pid.i_max = 3;
    assert(mcl_config_validate(&cfg) == MCL_OK);
    cfg.avs_stop_voltage = 36;
    assert(mcl_config_validate(&cfg) == MCL_ERR_PARAM);
    cfg.avs_stop_voltage = 30;
    hal.adc_read_phase = phase; hal.adc_read_bus = bus;
    hal.enc_read_speed = speed; hal.pwm_set_duty = pwm;
    assert(mcl_init(&motor, &cfg, &hal, &f, NULL, NULL, NULL) == MCL_OK);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORED);
    mcl_set_current(&motor, -3);
    mcl_start(&motor);
    for (i=0; i<2000; ++i) { mcl_control_tick(&motor); }
    assert(fabsf(motor.iq_applied + 3) < 0.001f);
    f.voltage = 29;
    mcl_control_tick(&motor);
    assert(fabsf(motor.iq_applied + 1.5f) < 0.001f);
    f.voltage = 30;
    mcl_control_tick(&motor);
    assert(motor.iq_applied == 0);
    assert(motor.iq_ref == -3); /* user command preserved */
    mcl_set_current(&motor, 2);
    mcl_control_tick(&motor);
    assert(motor.iq_applied == 2); /* motoring unaffected */
    f.speed = -500;
    mcl_control_tick(&motor);
    assert(motor.iq_applied == 0); /* reverse regeneration */
    f.speed = 0;
    mcl_control_tick(&motor);
    assert(motor.iq_applied == 0); /* direction deadband */
    f.voltage = 24;
    mcl_control_tick(&motor);
    assert(motor.avs_scale > 0 && motor.avs_scale < 0.01f);
    for (i=0; i<2000; ++i) { mcl_control_tick(&motor); }
    assert(motor.avs_scale == 1);
    f.voltage = 37;
    mcl_control_tick(&motor);
    assert(motor.fault == MCL_FAULT_OVERVOLTAGE);

    /* Exercise the actual decimated speed-loop path, not just helpers. */
    mcl_init(&motor, &cfg, &hal, &f, NULL, NULL, NULL);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORED);
    mcl_set_speed(&motor, 1000);
    mcl_start(&motor);
    f.speed = 600; f.voltage = 30;
    for (i=0; i<2000; ++i)
    {
        mcl_control_tick(&motor);
        assert(motor.iq_applied >= 0);
    }
    f.voltage = 29;
    for (i=0; i<2000; ++i)
    {
        mcl_control_tick(&motor);
        assert(motor.iq_applied >= -1.5001f);
    }
    assert(motor.iq_applied < 0);
    f.voltage = 30;
    mcl_control_tick(&motor);
    assert(motor.iq_applied == 0); /* fast guard between speed updates */

    /* Explicit AVS off preserves configured thresholds and speed anti-windup. */
    cfg.avs_enabled = false;
    assert(mcl_config_validate(&cfg) == MCL_OK);
    mcl_init(&motor, &cfg, &hal, &f, NULL, NULL, NULL);
    mcl_set_mode(&motor, MCL_MODE_FOC_SENSORED);
    mcl_set_current(&motor, -3);
    mcl_start(&motor);
    f.speed = 600; f.voltage = 30;
    mcl_control_tick(&motor);
    assert(motor.iq_applied == -3);
    assert(motor.avs_scale == 1);
    mcl_set_speed(&motor, 1000);
    f.speed = 0; f.voltage = 24;
    for (i=0; i<40000; ++i) { mcl_control_tick(&motor); }
    assert(motor.pid_speed.i_term < -1.0f);
    assert(motor.iq_applied <= 3);
    cfg.speed_aw_time = NAN;
    assert(mcl_config_validate(&cfg) == MCL_ERR_PARAM);
    cfg.speed_aw_time = 0;
    assert(mcl_config_validate(&cfg) == MCL_OK);
    cfg.avs_enabled = true;
    assert(mcl_config_validate(&cfg) == MCL_ERR_PARAM);
    cfg.speed_aw_time = 0.02f;
    cfg.avs_start_voltage = NAN;
    assert(mcl_config_validate(&cfg) == MCL_ERR_PARAM);

    mcl_pid_init(&pid, &cfg.speed_pid);
    for (i=0; i<10000; ++i)
    {
        value = mcl_speed_pid_limited(&pid, 1000, 0.001f, -3, 3, 0.001f/0.021f);
        assert(value <= 3 && value >= -3);
    }
    assert(pid.i_term < 0); /* no accumulated +3A after prolonged saturation */
    mcl_pid_reset(&pid);
    for (i=0; i<10000; ++i)
    {
        value = mcl_speed_pid_limited(&pid, -200, 0.001f, 0, 3, 0.001f/0.021f);
        assert(value == 0); /* blocked braking cannot wind up negative */
    }
    value = mcl_speed_pid_limited(&pid, -200, 0.001f, -3, 3, 0.001f/0.021f);
    assert(value > -0.02f); /* release has no accumulated braking pulse */
    puts("AVS: limits, both directions, deadband, recovery, hard trip, anti-windup PASS");
    return 0;
}
