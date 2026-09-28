#ifndef MCL_AVS_H
#define MCL_AVS_H

#include "mcl_pid.h"
#include <math.h>

/* Voltages/speeds use host units; step and gain are dimensionless. */
static inline mcl_scalar mcl_avs_scale_update(mcl_scalar previous, mcl_scalar voltage,
                                        mcl_scalar start, mcl_scalar stop,
                                        mcl_scalar step)
{
    mcl_scalar target;
#if !defined(MCL_USE_Q15) && !defined(MCL_USE_Q31)
    if (!isfinite(voltage)) { return (mcl_scalar)0; }
#endif
    if (voltage >= stop) { target = (mcl_scalar)0; }
    else if (voltage <= start) { target = MCL_FROM_FLOAT(1.0f); }
    else { target = MCL_DIV(MCL_SUB(stop, voltage), MCL_SUB(stop, start)); }
    if (target <= previous) { return target; }
    previous = MCL_ADD(previous, step);
    return previous < target ? previous : target;
}

static inline void mcl_avs_bounds(mcl_scalar speed, mcl_scalar deadband, mcl_scalar scale,
                                  mcl_scalar *lower, mcl_scalar *upper)
{
    /* Near zero, constrain both signs: avoid choosing a noisy direction. */
    if (scale == MCL_FROM_FLOAT(1.0f)) { return; }
    if (speed >= MCL_NEG(deadband)) { *lower = MCL_MUL(*lower, scale); }
    if (speed <= deadband) { *upper = MCL_MUL(*upper, scale); }
}

#if !defined(MCL_USE_Q15) && !defined(MCL_USE_Q31)
static inline float mcl_speed_pid_limited(mcl_pid *pid, float error, float dt,
                                         float lower, float upper, float aw_gain)
{
    float delta = pid->params.ki * error * dt;
    float pd = pid->params.kp * error +
               pid->params.kd * (error - pid->prev_error);
    float candidate = pid->i_term + delta;
    float unsat;
    float output;
    if (candidate > pid->params.i_max) { candidate = pid->params.i_max; }
    if (candidate < pid->params.i_min) { candidate = pid->params.i_min; }
    unsat = pd + candidate;
    /* Stop integrating into saturation; still permit unwinding. */
    if (!((unsat > upper && delta > 0.0f) ||
          (unsat < lower && delta < 0.0f)))
    {
        pid->i_term = candidate;
    }
    unsat = pd + pid->i_term;
    output = unsat > upper ? upper : (unsat < lower ? lower : unsat);
    pid->i_term += aw_gain * (output - unsat);
    if (pid->i_term > pid->params.i_max) { pid->i_term = pid->params.i_max; }
    if (pid->i_term < pid->params.i_min) { pid->i_term = pid->params.i_min; }
    pid->prev_error = error;
    pid->prev_out = output;
    return output;
}
#else
#if defined(MCL_USE_Q15)
#define MCL_AVS_ONE INT64_C(32768)
#else
#define MCL_AVS_ONE INT64_C(2147483648)
#endif

/* Split before multiplication: a may exceed the scalar range, e.g. P+I+D.
   This avoids overflow even for Q31 full-scale derivative transients. */
static inline int64_t mcl_avs_wide_mul(int64_t a, int64_t b)
{
    return (a / MCL_AVS_ONE) * b + (a % MCL_AVS_ONE) * b / MCL_AVS_ONE;
}

static inline mcl_scalar mcl_speed_pid_limited(mcl_pid *pid, mcl_scalar error,
    mcl_scalar dt, mcl_scalar lower, mcl_scalar upper, mcl_scalar aw_gain)
{
#if defined(MCL_USE_Q15)
    const int64_t integral_scale = MCL_AVS_ONE * MCL_AVS_ONE;
    int64_t rate = (int64_t)pid->params.ki * error;
#else
    const int64_t integral_scale = MCL_AVS_ONE;
    int64_t rate = mcl_avs_wide_mul(pid->params.ki, error);
#endif
    int64_t fraction = rate * dt + pid->aw_remainder;
    int64_t delta = fraction / integral_scale;
    int64_t pd = mcl_avs_wide_mul(pid->params.kp, error) +
        mcl_avs_wide_mul(pid->params.kd, (int64_t)error - pid->prev_error);
    int64_t candidate = (int64_t)pid->i_term + delta;
    int64_t unsat;
    int64_t output;
    int64_t correction;
    int64_t next;
    if (candidate > pid->params.i_max) { candidate = pid->params.i_max; }
    if (candidate < pid->params.i_min) { candidate = pid->params.i_min; }
    unsat = pd + candidate;
    if (!((unsat > upper && rate > 0) || (unsat < lower && rate < 0)))
    {
        pid->i_term = (mcl_scalar)candidate;
        pid->aw_remainder = fraction % integral_scale;
    }
    else { pid->aw_remainder = 0; }
    unsat = pd + pid->i_term;
    output = unsat > upper ? upper : (unsat < lower ? lower : unsat);
    correction = output - unsat;
    next = (int64_t)pid->i_term + mcl_avs_wide_mul(correction, aw_gain);
    pid->aw_tracking_remainder += ((correction % MCL_AVS_ONE) * aw_gain) % MCL_AVS_ONE;
    next += pid->aw_tracking_remainder / MCL_AVS_ONE;
    pid->aw_tracking_remainder %= MCL_AVS_ONE;
    if (next > pid->params.i_max)
    { next = pid->params.i_max; pid->aw_remainder = pid->aw_tracking_remainder = 0; }
    if (next < pid->params.i_min)
    { next = pid->params.i_min; pid->aw_remainder = pid->aw_tracking_remainder = 0; }
    pid->i_term = (mcl_scalar)next;
    pid->prev_error = error;
    pid->prev_out = (mcl_scalar)output;
    return (mcl_scalar)output;
}
#undef MCL_AVS_ONE
#endif
#endif
