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
#include <math.h>

/* ld<=0 或等于 Lq 时返回永磁磁链，隐极路径与改前一致。 */
static mcl_scalar flux_active(const mcl_observer_flux *self,
                              mcl_scalar i_alpha, mcl_scalar i_beta,
                              mcl_scalar theta)
{
    float ld = (float)MCL_TO_FLOAT(self->params.ld);
    float lq = (float)MCL_TO_FLOAT(self->params.inductance);
    float lambda = (float)MCL_TO_FLOAT(self->params.lambda);
    float id;
    float active;

    if (ld <= 0.0f || ld == lq)
    {
        return self->params.lambda;
    }
    {
        float th = (float)MCL_TO_FLOAT(theta);
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
        th *= 6.28318530718f; /* atan2 定点返回圈数 */
#endif
        id = (float)MCL_TO_FLOAT(i_alpha) * cosf(th) +
             (float)MCL_TO_FLOAT(i_beta) * sinf(th);
    }
    active = lambda + (ld - lq) * id;
    if (active < lambda * 0.2f)
    {
        active = lambda * 0.2f;
    }
    return MCL_FROM_FLOAT(active);
}

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
        mcl_scalar lambda_nom = flux_active(self, i_alpha, i_beta,
                                            mcl_math_atan2(lambda_beta, lambda_alpha));
        mcl_scalar err = MCL_SUB(self->lambda_est, lambda_nom);
        mcl_scalar ux = MCL_DIV(lambda_alpha, self->lambda_est);
        mcl_scalar uy = MCL_DIV(lambda_beta, self->lambda_est);
        mcl_scalar corr = MCL_MUL(MCL_MUL(self->params.gain, err), dt);
        self->x1 = MCL_SUB(self->x1, MCL_MUL(ux, corr));
        self->x2 = MCL_SUB(self->x2, MCL_MUL(uy, corr));
    }

    /* 相位 = atan2(λ_β, λ_α)（转子磁链角）。
       历史教训：曾加 −40° 经验修正——实为 R 参数误差（0.46 vs 真值 0.69）
       造成的稳态角偏差（ΔR·iq/ω/|λ| ≈ 50°），并非采样/调制残差；
       R 用真值后自然输出即转子磁链，无需任何固定修正角。 */
    if (phase_rad != NULL)
    {
        *phase_rad = mcl_math_atan2(lambda_beta, lambda_alpha);
    }

    /* 保存本拍电流（seed 换算定子磁链用）。
       历史 bug：未保存导致 seed 的 L*i_last 恒 0，拖动期间 x = λ_raw 缺
       L*i，下一拍 λ = x − L*i 被 L*i≈0.006Wb 大幅旋转（vs λ=0.0036），
       PLL 输入乱跳 → 切闭环必崩（实测拖动期 est−λout=+160° 而非 −90°）。 */
    self->i_alpha_last = i_alpha;
    self->i_beta_last = i_beta;

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

    /* 内部状态 x1/x2 是定子磁链 ψ_s = 转子磁链 λ_r + L*i。
       传入矢量按永磁磁链幅值时，凸极再缩放到有功磁链。 */
    {
        float pm = (float)MCL_TO_FLOAT(self->params.lambda);
        mcl_scalar theta = mcl_math_atan2(flux_beta, flux_alpha);
        mcl_scalar active = flux_active(self, self->i_alpha_last,
                                        self->i_beta_last, theta);
        float scale = 1.0f;
        if (pm > 1.0e-8f)
        {
            scale = (float)MCL_TO_FLOAT(active) / pm;
        }
        self->x1 = MCL_ADD(MCL_MUL(flux_alpha, MCL_FROM_FLOAT(scale)),
                           MCL_MUL(self->params.inductance, self->i_alpha_last));
        self->x2 = MCL_ADD(MCL_MUL(flux_beta, MCL_FROM_FLOAT(scale)),
                           MCL_MUL(self->params.inductance, self->i_beta_last));
        self->lambda_est = active;
    }
}

const mcl_observer_ops mcl_observer_flux_ops = {
    .init = flux_init,
    .reset = flux_reset,
    .update = flux_update,
    .seed = flux_seed,
    .get_confidence = NULL
};

#endif /* MCL_DISABLE_OBSERVER */
