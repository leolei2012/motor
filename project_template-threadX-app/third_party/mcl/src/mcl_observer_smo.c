/**
 * SMO with current prediction, sliding correction and two EMF filters.
 * Input convention: previous interval voltage and current sample at its end.
 * Retains the AN1078 E+Z feedback structure. Consequently Efinal is a
 * filtered internal state, NOT the full physical back-EMF amplitude.
 *
 * Float uses V/A/s/radians; fixed point uses per-unit and angle in turns.
 * The phase correction H^2/(1+H) is a sliding-regime approximation, so
 * finite observer gain, parameter error and inverter dead time still need
 * hardware validation. See docs/smo_validation.md for regression limits.
 */
#include "mcl_observer_smo.h"
#include "mcl_math.h"
#include <math.h>

#ifndef MCL_DISABLE_OBSERVER

static void smo_reset(void *impl);

static void smo_init(void *impl, const void *params)
{
    mcl_observer_smo *self = (mcl_observer_smo *)impl;
    if (self == NULL)
    {
        return;
    }

    if (params != NULL)
    {
        self->params = *(const mcl_observer_smo_params *)params;
    }

    smo_reset(impl);
}

static void smo_reset(void *impl)
{
    mcl_observer_smo *self = (mcl_observer_smo *)impl;
    if (self == NULL)
    {
        return;
    }

    self->i_alpha_hat = (mcl_scalar)0;
    self->i_beta_hat = (mcl_scalar)0;
    self->e_alpha = (mcl_scalar)0;
    self->e_beta = (mcl_scalar)0;
    self->e_alpha_final = (mcl_scalar)0;
    self->e_beta_final = (mcl_scalar)0;
    self->w_est = (mcl_scalar)0;
    self->z_alpha = (mcl_scalar)0;
    self->z_beta = (mcl_scalar)0;
    self->theta_prev = (mcl_scalar)0;
    self->dtheta_prev = (mcl_scalar)0;
    self->seed_omega = (mcl_scalar)0;
    self->filter_step = (mcl_scalar)0;
    self->phase = (mcl_scalar)0;
    self->dt = (mcl_scalar)0;
    self->i_alpha_last = (mcl_scalar)0;
    self->i_beta_last = (mcl_scalar)0;
}

void mcl_observer_smo_set_seed_omega(void *impl, mcl_scalar omega)
{
    mcl_observer_smo *self = (mcl_observer_smo *)impl;
    if (self == NULL)
    {
        return;
    }
    self->seed_omega = omega;
}

/* AN1078 滑模控制：线性区 Z = Kslide·Ierr/MaxSMCError；饱和区 Z = ±Kslide。
   输入 x = 电流误差，boundary = MaxSMCError，输出 = Z/Kslide ∈ [-1,1]。
   （与 AN1078 CalcZalpha 的「比例项再乘 Kslide」等价，此处把除以 boundary 与
   乘以 Kslide 拆开，避免 MCL_DIV 的定点精度损失。） */
static mcl_scalar smo_slide_component(mcl_scalar err, mcl_scalar boundary)
{
    mcl_scalar r;
    if (boundary <= (mcl_scalar)0)
    {
        /* 退化：无线性区，纯开关 */
        if (err > (mcl_scalar)0) { return MCL_FROM_FLOAT(1.0f); }
        if (err < (mcl_scalar)0) { return MCL_FROM_FLOAT(-1.0f); }
        return (mcl_scalar)0;
    }
    if (err > boundary) { return MCL_FROM_FLOAT(1.0f); }
    if (err < MCL_NEG(boundary)) { return MCL_FROM_FLOAT(-1.0f); }
    r = MCL_DIV(err, boundary);
    if (r > MCL_FROM_FLOAT(1.0f)) { return MCL_FROM_FLOAT(1.0f); }
    if (r < MCL_FROM_FLOAT(-1.0f)) { return MCL_FROM_FLOAT(-1.0f); }
    return r;
}

/* Convert only angles to/from float radians; the electrical model stays in
 * mcl_scalar. This also avoids representing pi or 2*pi in Q1.15/Q1.31. */
static float smo_radians(mcl_scalar angle)
{
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    return MCL_TO_FLOAT(angle) * MCL_TWO_PI;
#else
    return angle;
#endif
}

static mcl_scalar smo_angle(float angle)
{
    while (angle > MCL_PI) { angle -= MCL_TWO_PI; }
    while (angle < -MCL_PI) { angle += MCL_TWO_PI; }
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    return MCL_FROM_FLOAT(angle * MCL_INV_TWO_PI);
#else
    return angle;
#endif
}

static void smo_update(void *impl, mcl_scalar v_alpha, mcl_scalar v_beta,
                       mcl_scalar i_alpha, mcl_scalar i_beta, mcl_scalar dt,
                       mcl_scalar *phase_rad, mcl_scalar *speed_rad_s)
{
    mcl_observer_smo *self = (mcl_observer_smo *)impl;
    mcl_scalar g, f, z_a, z_b, kslf;
    float raw, delta, step, target, minimum, direction, ratio, correction;
    if (self == NULL || dt <= (mcl_scalar)0 ||
        self->params.inductance <= (mcl_scalar)0)
    {
        if (phase_rad != NULL) { *phase_rad = (mcl_scalar)0; }
        if (speed_rad_s != NULL) { *speed_rad_s = (mcl_scalar)0; }
        return;
    }
    self->dt = dt;

    /* Predict i_hat[k] with voltage[k-1] and correction[k-1], THEN compare
     * with i[k]. Comparing i_hat[k-1] with i[k] injects a false L*di/dt EMF. */
    g = MCL_DIV(dt, self->params.inductance);
    f = MCL_SUB(MCL_FROM_FLOAT(1.0f), MCL_MUL(self->params.resistance, g));
    self->i_alpha_hat = MCL_ADD(MCL_MUL(f, self->i_alpha_hat),
        MCL_MUL(g, MCL_SUB(MCL_SUB(v_alpha, self->e_alpha), self->z_alpha)));
    self->i_beta_hat = MCL_ADD(MCL_MUL(f, self->i_beta_hat),
        MCL_MUL(g, MCL_SUB(MCL_SUB(v_beta, self->e_beta), self->z_beta)));
    z_a = MCL_MUL(self->params.gain,
        smo_slide_component(MCL_SUB(self->i_alpha_hat, i_alpha), self->params.boundary));
    z_b = MCL_MUL(self->params.gain,
        smo_slide_component(MCL_SUB(self->i_beta_hat, i_beta), self->params.boundary));
    self->z_alpha = z_a;
    self->z_beta = z_b;

    /* A nonzero signed hint is an explicit open-loop override. The caller
     * releases it with zero; a single noisy delta must not release it.
     * w_est is measured from RAW angle, excluding changing compensation. */
    step = MCL_TO_FLOAT(self->w_est);
    if (self->seed_omega != (mcl_scalar)0)
    {
        step = MCL_TO_FLOAT(self->seed_omega) * MCL_TO_FLOAT(dt);
    }
    target = step < 0.0f ? -step : step;
    minimum = MCL_TO_FLOAT(self->params.lpf) * MCL_TO_FLOAT(dt);
    if (minimum < 0.00001f) { minimum = 0.00001f; }
    if (target < minimum) { target = minimum; }
    if (target > 0.5f) { target = 0.5f; }
    /* Slow bandwidth adaptation avoids the positive feedback between
     * filter phase lag -> differentiated speed -> filter bandwidth.
     * At 16 kHz this is about 62.5ms; raw-speed smoothing is about 3ms. */
    self->filter_step = MCL_ADD(self->filter_step,
        MCL_MUL(MCL_FROM_FLOAT(0.001f), MCL_SUB(MCL_FROM_FLOAT(target), self->filter_step)));
    if (self->filter_step < MCL_FROM_FLOAT(minimum))
    {
        self->filter_step = MCL_FROM_FLOAT(minimum > 0.5f ? 0.5f : minimum);
    }
    kslf = self->filter_step;
    self->e_alpha = MCL_ADD(self->e_alpha, MCL_MUL(kslf, MCL_SUB(z_a, self->e_alpha)));
    self->e_beta = MCL_ADD(self->e_beta, MCL_MUL(kslf, MCL_SUB(z_b, self->e_beta)));
    self->e_alpha_final = MCL_ADD(self->e_alpha_final,
        MCL_MUL(kslf, MCL_SUB(self->e_alpha, self->e_alpha_final)));
    self->e_beta_final = MCL_ADD(self->e_beta_final,
        MCL_MUL(kslf, MCL_SUB(self->e_beta, self->e_beta_final)));

    raw = smo_radians(mcl_math_atan2(MCL_NEG(self->e_alpha_final), self->e_beta_final));
    delta = raw - smo_radians(self->theta_prev);
    if (delta > MCL_PI) { delta -= MCL_TWO_PI; }
    if (delta < -MCL_PI) { delta += MCL_TWO_PI; }
    self->theta_prev = smo_angle(raw);
    if (delta > MCL_PI / 3.0f) { delta = MCL_PI / 3.0f; }
    if (delta < -MCL_PI / 3.0f) { delta = -MCL_PI / 3.0f; }
    self->dtheta_prev = MCL_FROM_FLOAT(delta);
    self->w_est = MCL_ADD(self->w_est,
        MCL_MUL(MCL_FROM_FLOAT(0.02f), MCL_SUB(self->dtheta_prev, self->w_est)));

    /* With E fed back in the predictor, the ideal sliding transfer is
     * Efinal/Etrue = H^2/(1+H), not H^2. In the continuous approximation,
     * H=1/(1+j*r): lag=atan(r)+atan(r/2)=atan2(3*r,2-r*r).
     * 71.565 degrees applies ONLY at r=1. Use signed compensation and
     * remove the pi reversal in back-EMF when omega is negative. */
    direction = step < 0.0f ? -1.0f : 1.0f;
    ratio = (step < 0.0f ? -step : step) / MCL_TO_FLOAT(kslf);
    if (ratio > 10.0f) { ratio = 10.0f; }
    correction = atan2f(3.0f * ratio, 2.0f - ratio * ratio);
    self->phase = smo_angle(raw + direction * correction + (step < 0.0f ? MCL_PI : 0.0f));
    self->i_alpha_last = i_alpha;
    self->i_beta_last = i_beta;
    if (phase_rad != NULL) { *phase_rad = self->phase; }
    if (speed_rad_s != NULL) { *speed_rad_s = MCL_DIV(self->w_est, dt); }
}

static void smo_seed(void *impl, mcl_scalar flux_alpha, mcl_scalar flux_beta)
{
    mcl_observer_smo *self = (mcl_observer_smo *)impl;
    float omega, theta, magnitude, direction, angle;
    if (self == NULL) { return; }
    omega = MCL_TO_FLOAT(self->seed_omega);
    direction = omega < 0.0f ? -1.0f : 1.0f;
    theta = smo_radians(mcl_math_atan2(flux_beta, flux_alpha));
    magnitude = sqrtf(MCL_TO_FLOAT(flux_alpha) * MCL_TO_FLOAT(flux_alpha) +
                      MCL_TO_FLOAT(flux_beta) * MCL_TO_FLOAT(flux_beta)) * omega;
    /* ld>0 且不等于 Lq 时，幅值按有功磁链 λ+(Ld-Lq)·id。id=0 或隐极时与 λ·ω 相同。 */
    {
        float lq = MCL_TO_FLOAT(self->params.inductance);
        float ld = MCL_TO_FLOAT(self->params.ld);
        float lambda_pm = MCL_TO_FLOAT(self->params.flux);
        float th = smo_radians(self->phase);
        float id;
        float lambda_a;
        if (ld > 0.0f && lq > 0.0f && ld != lq && lambda_pm > 1.0e-8f)
        {
            id = MCL_TO_FLOAT(self->i_alpha_last) * cosf(th) +
                 MCL_TO_FLOAT(self->i_beta_last) * sinf(th);
            lambda_a = lambda_pm + (ld - lq) * id;
            if (lambda_a < lambda_pm * 0.2f)
            {
                lambda_a = lambda_pm * 0.2f;
            }
            magnitude *= lambda_a / lambda_pm;
        }
    }
    /* Consistent two-stage steady-state seed at |omega|/cutoff=1:
     * E1/Etrue=1/(2+j), E2/Etrue=1/((1+j)(2+j)), Z=Etrue-E1.
     * Do not seed both filters to the full physical EMF. */
    angle = theta - direction * 0.46364761f;
    self->e_alpha = MCL_FROM_FLOAT(-magnitude * 0.44721360f * sinf(angle));
    self->e_beta = MCL_FROM_FLOAT(magnitude * 0.44721360f * cosf(angle));
    angle = theta - direction * 1.24904577f;
    self->e_alpha_final = MCL_FROM_FLOAT(-magnitude * 0.31622777f * sinf(angle));
    self->e_beta_final = MCL_FROM_FLOAT(magnitude * 0.31622777f * cosf(angle));
    self->z_alpha = MCL_SUB(MCL_FROM_FLOAT(-magnitude * sinf(theta)), self->e_alpha);
    self->z_beta = MCL_SUB(MCL_FROM_FLOAT(magnitude * cosf(theta)), self->e_beta);
    self->w_est = MCL_MUL(self->seed_omega, self->dt);
    self->filter_step = MCL_ABS(self->w_est);
    if (self->filter_step > MCL_FROM_FLOAT(0.5f)) { self->filter_step = MCL_FROM_FLOAT(0.5f); }
    self->theta_prev = mcl_math_atan2(MCL_NEG(self->e_alpha_final), self->e_beta_final);
    self->phase = smo_angle(theta);
    self->dtheta_prev = self->w_est;
}
const mcl_observer_ops mcl_observer_smo_ops = {
    .init = smo_init,
    .reset = smo_reset,
    .update = smo_update,
    .seed = smo_seed,
    .get_confidence = NULL
};

#endif /* MCL_DISABLE_OBSERVER */
