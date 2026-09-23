#include "mcl.h"
#include <math.h>
#include <stdio.h>

typedef struct { float va, vb, obs_va, obs_vb; int invalid; } fixture;
static void pwm(void *ctx,mcl_scalar a,mcl_scalar b,mcl_scalar c)
{
    fixture *f=ctx;
    f->invalid |= !isfinite(a+b+c) || a<0 || b<0 || c<0 || a>0.95f || b>0.95f || c>0.95f;
    f->va=24.0f*(a-(a+b+c)/3.0f);
    f->vb=24.0f*(b-c)/sqrtf(3.0f);
}
static int adc(void *ctx,mcl_scalar *a,mcl_scalar *b,mcl_scalar *c)
{ (void)ctx; *a=*b=*c=0; return MCL_OK; }
static int bus(void *ctx,mcl_scalar *v,mcl_scalar *i)
{ (void)ctx; *v=24; *i=0; return MCL_OK; }
static void observe(void *ctx,mcl_scalar va,mcl_scalar vb,mcl_scalar ia,mcl_scalar ib,
                    mcl_scalar dt,mcl_scalar *angle,mcl_scalar *speed)
{
    fixture *f=ctx;
    (void)ia;(void)ib;(void)dt;
    f->obs_va=va;f->obs_vb=vb;
    if(angle) *angle=0;
    if(speed) *speed=0;
}
int main(void)
{
    mcl motor={0}; mcl_config cfg; mcl_hal_ops hal={0}; fixture f={0};
    mcl_observer_ops ops={0};
    int n,failed=0;
    mcl_config_default(&cfg); cfg.max_duty=0.95f; cfg.limits.enabled=0;
    hal.pwm_set_duty=pwm;hal.adc_read_phase=adc;hal.adc_read_bus=bus;ops.update=observe;
    mcl_init(&motor,&cfg,&hal,&f,&ops,&f,NULL);
    mcl_set_openloop_vf(&motor,0.8f,800);mcl_start(&motor);
    for(n=0;n<1000;++n)
    {
        float previous_a=f.va,previous_b=f.vb;
        /* Deliberately overmodulate to exercise clipping too. */
        if(n==500) motor.openloop_mag=3.0f;
        mcl_control_tick(&motor);
        if(fabsf(f.obs_va-previous_a)>0.00002f || fabsf(f.obs_vb-previous_b)>0.00002f) failed=1;
    }
    printf("Applied voltage reconstruction (linear + clipped): %s\n", failed||f.invalid?"FAIL":"PASS");
    return failed||f.invalid;
}
