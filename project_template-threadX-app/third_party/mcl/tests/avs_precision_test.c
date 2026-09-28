#include "mcl.h"
#include "mcl_avs.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define Q(x) MCL_FROM_FLOAT(x)
#define F(x) MCL_TO_FLOAT(x)
static mcl_scalar voltage, omega;
static int currents(void *ctx, mcl_scalar *a, mcl_scalar *b, mcl_scalar *c)
{ (void)ctx; *a=*b=*c=0; return MCL_OK; }
static int bus(void *ctx, mcl_scalar *v, mcl_scalar *i)
{ (void)ctx; *v=voltage; *i=0; return MCL_OK; }
static int speed(void *ctx, mcl_scalar *s)
{ (void)ctx; *s=omega; return MCL_OK; }
static void pwm(void *ctx, mcl_scalar a, mcl_scalar b, mcl_scalar c)
{ (void)ctx; (void)a; (void)b; (void)c; }

int main(void)
{
    mcl_scalar scale=Q(1), lo=Q(-0.8f), hi=Q(0.8f), out;
    mcl_pid pid;
    mcl_pid_params p={0};
    mcl_config cfg;
    mcl motor={0};
    mcl_hal_ops hal={0};
    int i, sign;
    scale=mcl_avs_scale_update(scale,Q(29.0f/64),Q(28.0f/64),Q(30.0f/64),Q(0.00125f));
    assert(fabsf(F(scale)-0.5f)<0.002f);
    mcl_avs_bounds(Q(0.4f),Q(0.01f),scale,&lo,&hi);
    assert(fabsf(F(lo)+0.4f)<0.002f && fabsf(F(hi)-0.8f)<0.002f);
    scale=0;
    for(i=0;i<800;i++) scale=mcl_avs_scale_update(scale,Q(0.375f),Q(0.4375f),Q(0.46875f),Q(0.00125f));
    assert(F(scale)>0.97f);
    for(sign=-1;sign<=1;sign+=2)
    {
        p.ki=Q(0.1f); p.i_min=p.out_min=Q(-0.8f); p.i_max=p.out_max=Q(0.8f);
        mcl_pid_init(&pid,&p);
        for(i=0;i<10000;i++)
            out=mcl_speed_pid_limited(&pid,Q(sign*0.01f),Q(0.001f),p.out_min,p.out_max,Q(0.05f));
        assert(fabsf(F(out)-sign*0.01f)<0.001f); /* sub-LSB integration */
    }
    p.kp=Q(0.9f); p.kd=Q(0.9f);
    mcl_pid_init(&pid,&p);
    for(i=0;i<10000;i++)
    {
        out=mcl_speed_pid_limited(&pid,Q(i&1 ? -1.0f:1.0f),Q(0.8f),Q(-0.2f),Q(0.2f),Q(0.9f));
        assert(out>=Q(-0.2f) && out<=Q(0.2f)); /* wide derivative/saturation */
    }
    p.kd=0;
    mcl_pid_init(&pid,&p);
    for(i=0;i<10000;i++)
    {
        out=mcl_speed_pid_limited(&pid,Q(-0.2f),Q(0.01f),0,Q(0.8f),Q(0.05f));
        assert(out==0);
    }
    out=mcl_speed_pid_limited(&pid,Q(-0.2f),Q(0.01f),Q(-0.8f),Q(0.8f),Q(0.05f));
    assert(F(out)>-0.003f);

    mcl_config_default(&cfg);
    cfg.avs_enabled=true; cfg.bus_voltage=Q(24.0f/64);
    cfg.avs_start_voltage=Q(28.0f/64); cfg.avs_stop_voltage=Q(30.0f/64);
    cfg.avs_speed_deadband=Q(0.01f); cfg.speed_aw_time=Q(0.02f);
    cfg.limits.overvoltage=Q(36.0f/64); cfg.limits.undervoltage=Q(8.0f/64);
    cfg.limits.enabled=MCL_PROTECT_OVERVOLTAGE; cfg.fault_stop_time=0;
    cfg.time_base=Q(0.01f); cfg.current_loop_freq_hz=16000; cfg.speed_loop_divider=10;
    assert(mcl_config_validate(&cfg)==MCL_OK);
    hal.adc_read_phase=currents; hal.adc_read_bus=bus; hal.enc_read_speed=speed; hal.pwm_set_duty=pwm;
    assert(mcl_init(&motor,&cfg,&hal,0,0,0,0)==MCL_OK);
    assert(fabsf(F(motor.dt)-0.00625f)<0.0001f);
    mcl_set_mode(&motor,MCL_MODE_FOC_SENSORED);
    mcl_set_current(&motor,Q(-0.8f)); mcl_start(&motor);
    voltage=Q(30.0f/64); omega=Q(0.4f);
    mcl_control_tick(&motor); assert(motor.iq_applied==0);
    mcl_set_current(&motor,Q(0.8f)); omega=Q(-0.4f);
    mcl_control_tick(&motor); assert(motor.iq_applied==0);
    mcl_set_speed(&motor,0); omega=Q(0.4f);
    for(i=0;i<2000;i++) { mcl_control_tick(&motor); assert(motor.iq_applied>=0); }
    voltage=Q(24.0f/64);
    for(i=0;i<2000;i++) mcl_control_tick(&motor);
    assert(motor.iq_applied<0);
    voltage=Q(37.0f/64); mcl_control_tick(&motor);
    assert(motor.fault==MCL_FAULT_OVERVOLTAGE);
    {
        mcl_scalar old_step=motor.avs_recovery_step;
        cfg.avs_recovery_time=Q(0.1f);
        assert(mcl_set_config(&motor,&cfg)==MCL_OK);
        assert(motor.avs_recovery_step<old_step);
        old_step=motor.avs_recovery_step;
        cfg.avs_stop_voltage=cfg.avs_start_voltage;
        assert(mcl_set_config(&motor,&cfg)==MCL_ERR_PARAM);
        assert(motor.avs_recovery_step==old_step);
    }
    puts("AVS precision: limits, recovery, sub-LSB integration, overflow, FOC, hard trip PASS");
    return 0;
}
