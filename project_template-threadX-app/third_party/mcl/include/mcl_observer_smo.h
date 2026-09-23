/**
 * Sliding-mode current observer. Predict with voltage[k-1], compare with
 * measured current[k], then update sliding correction and two EMF filters.
 * Float: V, A, s, rad. Fixed: per-unit, angle in turns.
 * E is fed back with Z; ideal Efinal/Etrue = H^2/(1+H). Compensation is
 * bandwidth- and direction-dependent (71.565 degrees only at |omega|/wc=1).
 */
#ifndef MCL_OBSERVER_SMO_H
#define MCL_OBSERVER_SMO_H

#include "mcl_types.h"
#include "mcl_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Parameters: gain is volts in float, voltage pu in fixed point. */
typedef struct
{
    mcl_scalar resistance;      /**< 相电阻 Ω（float）/ pu（定点） */
    mcl_scalar inductance;      /**< 相电感 H（float）/ pu（定点） */
    mcl_scalar flux;            /**< 永磁磁链 ψ_f Wb（float）/ pu（定点），接口一致用 */
    mcl_scalar gain;            /**< 滑模电压上限 Kslide，float: V / fixed: voltage pu */
    mcl_scalar lpf;             /**< 最低电气转速 rad/s（float）/ ω_pu（定点），滤波系数下限 */
    mcl_scalar boundary;        /**< 线性滑模区最大电流误差 MaxSMCError（AN1078=0.005） */
} mcl_observer_smo_params;

/**
 * @brief 滑模观测器实例
 */
typedef struct
{
    mcl_observer_smo_params params; /**< 参数 */
    mcl_scalar i_alpha_hat;         /**< 估计电流 α A */
    mcl_scalar i_beta_hat;          /**< 估计电流 β A */
    mcl_scalar e_alpha;             /**< 反电动势 α（一级滤波） */
    mcl_scalar e_beta;              /**< 反电动势 β（一级滤波） */
    mcl_scalar e_alpha_final;       /**< 反电动势 α（二级滤波，用于角度） */
    mcl_scalar e_beta_final;        /**< 反电动势 β（二级滤波，用于角度） */
    mcl_scalar w_est;               /**< 平滑的原始角增量 omega*dt（弧度，非 rad/s） */
    mcl_scalar theta_prev;          /**< 上一拍未补偿的 EMF 角（速度差分） */
    mcl_scalar dtheta_prev;         /**< 上一拍原始角度增量 omega*dt（弧度） */
    mcl_scalar dt;                  /**< 控制周期 s（seed 里反电动势换算 ω=w_est/dt 用） */
    mcl_scalar z_alpha;             /**< 诊断：本拍滑模输出 z_α（饱和/线性区） */
    mcl_scalar z_beta;              /**< 诊断：本拍滑模输出 z_β */
    mcl_scalar seed_omega;          /**< 有符号开环速度提示；0=释放到自适应估速 */
    mcl_scalar filter_step;         /**< 实际低通系数 [0, 0.5]，交接时平滑变化 */
    mcl_scalar phase;               /**< 补偿后的转子角；调试使用，不重复计算固定补偿 */
} mcl_observer_smo;

/**
 * @brief 滑模观测器接口（注入 mcl_observer 载体用）
 */
extern const mcl_observer_ops mcl_observer_smo_ops;

/**
 * @brief 设置开环有符号速度提示；调用方在切闭环时明确写 0 释放
 * @param impl  观测器实例（mcl_observer_smo*）
 * @param omega 开环电气角速度 rad/s / pu；正负均支持，0 释放
 */
void mcl_observer_smo_set_seed_omega(void *impl, mcl_scalar omega);

#ifdef __cplusplus
}
#endif

#endif /* MCL_OBSERVER_SMO_H */
