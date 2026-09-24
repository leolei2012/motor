/**
 * @file    mcl_observer_ortega.c
 * @brief   mcl 电机控制库：ORTEGA 型磁链观测器实现
 *
 * 参考 Ortega et al. 观测器（VESC FOC_OBSERVER_ORTEGA_ORIGINAL）：
 *
 *   err = λ² - |λ_r|²，λ_r = ψ_s - L*i
 *   if (err > 0) err = 0          （非对称 clamp，保证全局收敛）
 *   dψ_s/dt = v - R*i + (γ/2)·λ_r·err
 *   phase  = atan2(λ_r_β, λ_r_α)
 *
 * 相位由 PLL 估计，此处不直接输出；速度输出置 0。
 */

#include "mcl_observer_ortega.h"
#include "mcl_math.h"
#include <math.h>

/* ld<=0 或等于 Lq 时返回永磁磁链，隐极路径与改前一致。 */
static mcl_scalar ortega_active_flux(const mcl_observer_ortega *self,
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

static void ortega_reset(void *impl);

static void ortega_init(void *impl, const void *params)
{
    mcl_observer_ortega *self = (mcl_observer_ortega *)impl;
    if (self == NULL)
    {
        return;
    }

    if (params != NULL)
    {
        self->params = *(const mcl_observer_ortega_params *)params;
    }

    ortega_reset(impl);
}

static void ortega_reset(void *impl)
{
    mcl_observer_ortega *self = (mcl_observer_ortega *)impl;
    if (self == NULL)
    {
        return;
    }

    self->x1 = (mcl_scalar)0;
    self->x2 = (mcl_scalar)0;
    self->lambda_est = (mcl_scalar)0;
    self->i_alpha_last = (mcl_scalar)0;
    self->i_beta_last = (mcl_scalar)0;
    /* 电阻自适应：初始 r_est_state = 配置相电阻，r_est 随温度漂移由观测器实时修正 */
    self->r_est_state = self->params.resistance;
    self->r_est = self->params.resistance;
    self->speed = (mcl_scalar)0;
}

static void ortega_update(void *impl, mcl_scalar v_alpha, mcl_scalar v_beta,
                          mcl_scalar i_alpha, mcl_scalar i_beta, mcl_scalar dt,
                          mcl_scalar *phase_rad, mcl_scalar *speed_rad_s)
{
    mcl_observer_ortega *self = (mcl_observer_ortega *)impl;
    mcl_scalar R = self->params.resistance;   /* 固定 R：电阻自适应回填实测导致磁链/速度震荡，暂回退 */
    mcl_scalar L = self->params.inductance;
    mcl_scalar lambda = self->params.lambda;
    mcl_scalar gamma_half;
    mcl_scalar L_ia;
    mcl_scalar L_ib;
    mcl_scalar lambda_alpha;
    mcl_scalar lambda_beta;
    mcl_scalar err;

    if (self == NULL)
    {
        return;
    }

    /* 动态 gain 缩放（对齐 VESC mcpwm_foc.c 4150-4158 行）：
       VESC 用 |duty|∈[0, 40/v_bus] 映射 gamma∈[0, gain]，再 clamp 到 gain_slow×gain，
       最后 ×4。目的：低速（电压幅值小）时反电动势信号弱，全量 gain 会让幅值反馈
       在磁链上过驱动震荡；高速时需要全量 gain 压住积分慢漂。
       这里用电压幅值 |v|=√(vα²+vβ²) 替代 duty（两者成正比：相电压≈duty·vbus/2），
       无需额外传 duty。v_max 取 vbus/2（此时 scale→1 全量）。 */
    {
        mcl_scalar v_mag = mcl_math_sqrt(MCL_ADD(MCL_MUL(v_alpha, v_alpha),
                                                  MCL_MUL(v_beta, v_beta)));
        mcl_scalar v_norm = MCL_DIV(v_mag, MCL_FROM_FLOAT(12.0f)); /* vbus/2=12V */
        mcl_scalar gain_slow = MCL_FROM_FLOAT(0.05f);              /* 低速下限比例 */
        mcl_scalar gamma_scale;
        if (v_norm > MCL_FROM_FLOAT(1.0f))
        {
            v_norm = MCL_FROM_FLOAT(1.0f);
        }
        if (v_norm < gain_slow)
        {
            v_norm = gain_slow;
        }
        gamma_scale = v_norm;   /* gain 按电压幅值线性缩放，下限 0.05 */
        gamma_half = MCL_MUL(MCL_MUL(self->params.gain, gamma_scale), MCL_FROM_FLOAT(0.5f));
    }
    L_ia = MCL_MUL(L, i_alpha);
    L_ib = MCL_MUL(L, i_beta);

    /* 估计转子磁链（更新前）。凸极时收敛目标是有功磁链，不是固定永磁磁链。 */
    lambda_alpha = MCL_SUB(self->x1, L_ia);
    lambda_beta = MCL_SUB(self->x2, L_ib);
    lambda = ortega_active_flux(self, i_alpha, i_beta,
                                mcl_math_atan2(lambda_beta, lambda_alpha));

    /* 幅值平方误差：err = |λ_nom|² − |λ_est|²，VESC 原版「非对称 clamp」：
       err>0（磁链偏小）时置 0 —— 这是 Ortega 论文（Bernard-Praly 2017）的收敛性
       要求：只在上限负反馈压小，不往下限正反馈拉大。此前误改成「双向反馈」，
       导致磁链偏小时正反馈与积分项耦合 → 磁链/角度慢漂 → 速度环慢摆 24s 一次
       掉速重拖。恢复 VESC 原版非对称 clamp 以消除慢漂。 */
    err = MCL_SUB(MCL_MUL(lambda, lambda),
                  MCL_ADD(MCL_MUL(lambda_alpha, lambda_alpha),
                          MCL_MUL(lambda_beta, lambda_beta)));
    if (err > (mcl_scalar)0)
    {
        err = (mcl_scalar)0;
    }

    /* 定子磁链积分 + 沿 λ_r 方向的幅值反馈 */
    self->x1 = MCL_ADD(self->x1,
        MCL_MUL(MCL_ADD(MCL_SUB(v_alpha, MCL_MUL(R, i_alpha)),
                        MCL_MUL(MCL_MUL(gamma_half, lambda_alpha), err)), dt));
    self->x2 = MCL_ADD(self->x2,
        MCL_MUL(MCL_ADD(MCL_SUB(v_beta, MCL_MUL(R, i_beta)),
                        MCL_MUL(MCL_MUL(gamma_half, lambda_beta), err)), dt));

    /* 更新后转子磁链 + 幅值 */
    lambda_alpha = MCL_SUB(self->x1, L_ia);
    lambda_beta = MCL_SUB(self->x2, L_ib);
    self->lambda_est = mcl_math_sqrt(MCL_ADD(MCL_MUL(lambda_alpha, lambda_alpha),
                                              MCL_MUL(lambda_beta, lambda_beta)));

    /* 防磁链幅值崩溃（对齐 VESC foc_math.c：mag < λ·0.5 时 ×1.1 拉回）。
       纯积分会让定子磁链幅值漂移到接近 0，使 atan2 角度噪声极大、极易失步；
       幅值过低时按 1.1 倍（= x + 0.1x，0.1<1 定点可表达）拉回，抑制漂移。 */
    {
        mcl_scalar mag_psi = mcl_math_sqrt(MCL_ADD(MCL_MUL(self->x1, self->x1),
                                                   MCL_MUL(self->x2, self->x2)));
        if (mag_psi < MCL_MUL(lambda, MCL_FROM_FLOAT(0.5f)))
        {
            self->x1 = MCL_ADD(self->x1, MCL_MUL(self->x1, MCL_FROM_FLOAT(0.1f)));
            self->x2 = MCL_ADD(self->x2, MCL_MUL(self->x2, MCL_FROM_FLOAT(0.1f)));
        }
    }

    self->i_alpha_last = i_alpha;
    self->i_beta_last = i_beta;

    /* 电阻自适应观测器（对齐 VESC mcpwm_foc.c 4160-4173，论文 DOI 10.1002/acs.2587
       「An adaptive flux observer for the PMSM」）：基于功率平衡实时估计相电阻，
       抵消绕组温升导致的 R 慢漂（这是无温度传感器下「1 分钟级磁链慢漂→掉速」的根因）。
       公式：
         i_abs² = iα² + iβ²
         r_est = r_est_state − 0.5·g·L·i_abs²
         r_dot = −g·( r_est·i_abs² + ω·(iβ·x1 − iα·x2) − (iα·vα + iβ·vβ) )
         r_est_state += r_dot·dt，并 clamp 到 [R·0.5, R·2]
       g = 0.00002（VESC 默认），ω = self->speed（电气 rad/s，mcl.c 在 update 前写入）。 */
    {
        mcl_scalar g = MCL_FROM_FLOAT(0.00002f);
        mcl_scalar i_abs_sq = MCL_ADD(MCL_MUL(i_alpha, i_alpha), MCL_MUL(i_beta, i_beta));
        mcl_scalar r_est_now = MCL_SUB(self->r_est_state,
            MCL_MUL(MCL_MUL(MCL_FROM_FLOAT(0.5f), MCL_MUL(g, L)), i_abs_sq));
        mcl_scalar p_term1 = MCL_MUL(r_est_now, i_abs_sq);
        mcl_scalar p_term2 = MCL_MUL(self->speed,
            MCL_SUB(MCL_MUL(i_beta, self->x1), MCL_MUL(i_alpha, self->x2)));
        mcl_scalar p_term3 = MCL_ADD(MCL_MUL(i_alpha, v_alpha), MCL_MUL(i_beta, v_beta));
        mcl_scalar r_dot = MCL_NEG(MCL_MUL(g,
            MCL_SUB(MCL_ADD(p_term1, p_term2), p_term3)));
        self->r_est_state = MCL_ADD(self->r_est_state, MCL_MUL(r_dot, dt));

        /* clamp r_est_state 到 [0.5R_nom, 2R_nom]，防发散 */
        {
            mcl_scalar lo = MCL_MUL(self->params.resistance, MCL_FROM_FLOAT(0.5f));
            mcl_scalar hi = MCL_MUL(self->params.resistance, MCL_FROM_FLOAT(2.0f));
            if (self->r_est_state < lo) { self->r_est_state = lo; }
            if (self->r_est_state > hi) { self->r_est_state = hi; }
        }
        self->r_est = r_est_now;
    }

    if (phase_rad != NULL)
    {
        *phase_rad = mcl_math_atan2(lambda_beta, lambda_alpha);
    }
    if (speed_rad_s != NULL)
    {
        *speed_rad_s = (mcl_scalar)0;
    }
}

static void ortega_seed(void *impl, mcl_scalar flux_alpha, mcl_scalar flux_beta)
{
    mcl_observer_ortega *self = (mcl_observer_ortega *)impl;
    if (self == NULL)
    {
        return;
    }

    /* 定子磁链 = 转子磁链 + L*i（用上一拍电流近似当前）。
       传入矢量按永磁磁链幅值时，凸极再缩放到有功磁链。 */
    {
        float pm = (float)MCL_TO_FLOAT(self->params.lambda);
        mcl_scalar theta = mcl_math_atan2(flux_beta, flux_alpha);
        mcl_scalar active = ortega_active_flux(self, self->i_alpha_last,
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

void mcl_observer_ortega_set_speed(void *impl, mcl_scalar speed)
{
    mcl_observer_ortega *self = (mcl_observer_ortega *)impl;
    if (self == NULL)
    {
        return;
    }
    self->speed = speed;
}

const mcl_observer_ops mcl_observer_ortega_ops = {
    .init = ortega_init,
    .reset = ortega_reset,
    .update = ortega_update,
    .seed = ortega_seed,
    .get_confidence = NULL
};

#endif /* MCL_DISABLE_OBSERVER */
