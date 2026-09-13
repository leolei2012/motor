/**
 * @file    mcl_observer_flux.c
 * @brief   mcl 电机控制库：磁链观测器参考实现
 *
 * 基础磁链观测器（电压积分形式）：
 *   1. 定子磁链积分：ψ_s += (v - R*i) * dt
 *   2. 转子磁链：λ_r = ψ_s - L*i
 *   3. 相位：phase = atan2(λ_β, λ_α)
 *
 * 作为观测器插槽的模板；速度由 PLL 估计，此处不直接输出。
 * gain 参数预留（用于后续幅值校正 / 泄漏积分器防漂移）。
 */

#include "mcl_observer_flux.h"
#include "mcl_math.h"

#ifndef MCL_DISABLE_OBSERVER

static void flux_reset(void *impl);

static void flux_init(void *impl, const void *params)
{
    mcl_observer_flux *self = (mcl_observer_flux *)impl;
    if (self == NULL)
    {
        return;
    }

    if (params != NULL)
    {
        self->params = *(const mcl_observer_flux_params *)params;
    }

    flux_reset(impl);
}

static void flux_reset(void *impl)
{
    mcl_observer_flux *self = (mcl_observer_flux *)impl;
    if (self == NULL)
    {
        return;
    }

    self->x1 = (mcl_scalar)0;
    self->x2 = (mcl_scalar)0;
    self->lambda_est = (mcl_scalar)0;
    self->i_alpha_last = (mcl_scalar)0;
    self->i_beta_last = (mcl_scalar)0;
}

static void flux_update(void *impl, mcl_scalar v_alpha, mcl_scalar v_beta,
                        mcl_scalar i_alpha, mcl_scalar i_beta, mcl_scalar dt,
                        mcl_scalar *phase_rad, mcl_scalar *speed_rad_s)
{
    mcl_observer_flux *self = (mcl_observer_flux *)impl;
    mcl_scalar lambda_alpha;
    mcl_scalar lambda_beta;

    if (self == NULL)
    {
        return;
    }

    /* 定子磁链积分：ψ_s += (v - R*i) * dt */
    self->x1 = MCL_ADD(self->x1,
                       MCL_MUL(MCL_SUB(v_alpha, MCL_MUL(self->params.resistance, i_alpha)), dt));
    self->x2 = MCL_ADD(self->x2,
                       MCL_MUL(MCL_SUB(v_beta, MCL_MUL(self->params.resistance, i_beta)), dt));

    /* 转子磁链：λ_r = ψ_s - L*i */
    lambda_alpha = MCL_SUB(self->x1, MCL_MUL(self->params.inductance, i_alpha));
    lambda_beta = MCL_SUB(self->x2, MCL_MUL(self->params.inductance, i_beta));

    /* 磁链幅值（诊断用） */
    self->lambda_est = mcl_math_sqrt(MCL_ADD(MCL_MUL(lambda_alpha, lambda_alpha),
                                              MCL_MUL(lambda_beta, lambda_beta)));

    /* 幅值误差反馈校正（ORTEGA 式）：沿磁链方向把幅值拉向标称 λ，
       消除纯积分直流漂移（gain 越大收敛越快；gain=0 关闭） */
    if (self->params.gain > (mcl_scalar)0 && self->lambda_est > (mcl_scalar)0)
    {
        mcl_scalar err = MCL_SUB(self->lambda_est, self->params.lambda);
        mcl_scalar ux = MCL_DIV(lambda_alpha, self->lambda_est);
        mcl_scalar uy = MCL_DIV(lambda_beta, self->lambda_est);
        mcl_scalar corr = MCL_MUL(MCL_MUL(self->params.gain, err), dt);
        self->x1 = MCL_SUB(self->x1, MCL_MUL(ux, corr));
        self->x2 = MCL_SUB(self->x2, MCL_MUL(uy, corr));
    }

    /* 相位 = atan2(λ_β, λ_α) − 90°
       实测（IF 开环 id=0/iq=1.2A，电流矢量在 θ+90°）：atan2 输出 = θ+167°，
       即 θ − λ_r = −167°。稳态稳定平衡要求转子磁链位于电流矢量后方、θ 前方：
       φ ∈ (θ, θ+90°)（力矩 T ∝ sin(γ−φ) 需为正且 dT/dφ<0），
       且 mcl 配置 openloop_seed_angle=π/2（λ_r = θ+90°，空载转子超前磁场 90°）。
       故正确输出 λ_r = θ+90°−α（α=负载角，实测 ≈13°），需减 90°：
       θ+167° − 90° = θ+77° = θ+90°−13°。 */
    if (phase_rad != NULL)
    {
        *phase_rad = mcl_math_atan2(lambda_beta, lambda_alpha);
        *phase_rad = MCL_SUB(*phase_rad, MCL_PI_2);
        if (*phase_rad > MCL_PI) { *phase_rad = MCL_SUB(*phase_rad, MCL_TWO_PI); }
        if (*phase_rad < MCL_NEG(MCL_PI)) { *phase_rad = MCL_ADD(*phase_rad, MCL_TWO_PI); }
    }

    /* 速度由 PLL 估计，此处不直接输出 */
    if (speed_rad_s != NULL)
    {
        *speed_rad_s = (mcl_scalar)0;
    }
}

static void flux_seed(void *impl, mcl_scalar flux_alpha, mcl_scalar flux_beta)
{
    mcl_observer_flux *self = (mcl_observer_flux *)impl;
    if (self == NULL)
    {
        return;
    }

    /* 内部约定与输出差 −90°（flux_update 输出 = atan2(λ_β, λ_α) − 90°）。
       seed API 给的是真实转子磁链 (flux_α, flux_β)，故先旋 +90° 存入内部：
       (λ_raw_α, λ_raw_β) = (−flux_β, flux_α)，输出即 = 真实磁链角 φ。
       内部 x1/x2 是定子磁链 ψ_s = λ_raw + L*i（用上一拍电流近似当前）。 */
    self->x1 = MCL_ADD(MCL_NEG(flux_beta),
                       MCL_MUL(self->params.inductance, self->i_alpha_last));
    self->x2 = MCL_ADD(flux_alpha,
                       MCL_MUL(self->params.inductance, self->i_beta_last));
    self->lambda_est = self->params.lambda;
}

const mcl_observer_ops mcl_observer_flux_ops = {
    .init = flux_init,
    .reset = flux_reset,
    .update = flux_update,
    .seed = flux_seed,
    .get_confidence = NULL
};

#endif /* MCL_DISABLE_OBSERVER */
