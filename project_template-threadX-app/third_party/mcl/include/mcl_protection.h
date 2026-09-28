/**
 * @file    mcl_protection.h
 * @brief   mcl 电机控制库：保护检测
 */

#ifndef MCL_PROTECTION_H
#define MCL_PROTECTION_H

#include "mcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** One control sample; values use the same physical/per-unit basis as limits.
 * current is the diagnostic current (Iq for FOC); ia/ib/ic drive detection. */
typedef struct
{
    mcl_scalar ia, ib, ic;
    mcl_scalar vbus, temp, speed, current;
    uint32_t tick;
} mcl_protection_sample;

/**
 * @brief 保护检测状态
 */
typedef struct
{
    mcl_protection_limits limits;   /**< 阈值 */
    mcl_protection_status status;   /**< 本拍宿主输入 */
    mcl_fault fault;               /**< Latched first fault; NONE after clear. */
    mcl_fault_info info;           /**< First-fault snapshot; retained after clear. */
    float recovery_time_s;         /**< 0 disables automatic clear. */
    float recovery_elapsed_s;      /**< Physical seconds in every precision. */
} mcl_protection;

/**
 * @brief 初始化保护
 * @param self   实例
 * @param limits 阈值
 */
void mcl_protection_init(mcl_protection *self, const mcl_protection_limits *limits);

/** 写入本拍保护输入。未调用时输入为 0。 */
void mcl_protection_set_status(mcl_protection *self, const mcl_protection_status *status);

/** Change configuration without discarding a latched fault or its snapshot. */
void mcl_protection_configure(mcl_protection *self,
                             const mcl_protection_limits *limits, float recovery_time_s);

/** Detect and latch a fault. No HAL access or PWM writes. */
mcl_fault mcl_protection_update(mcl_protection *self, const mcl_protection_sample *sample);

/** Latch an externally detected fault; repeated reports preserve the first event. */
int mcl_protection_assert(mcl_protection *self, mcl_fault fault,
                          const mcl_protection_sample *sample);
mcl_fault mcl_protection_get_fault(const mcl_protection *self);
int mcl_protection_get_fault_info(const mcl_protection *self, mcl_fault_info *out);

/** Explicit acknowledgement. Caller must ensure the motor is stopped.
 * Retains the historical snapshot; does not start the motor or reset hardware. */
void mcl_protection_clear(mcl_protection *self);

/** Advance the legacy automatic-clear delay in physical seconds.
 * Returns true only when the latch was cleared; caller returns motor to IDLE.
 * This is a timed acknowledgement, not proof that an external fault disappeared. */
bool mcl_protection_advance(mcl_protection *self, float dt_s);

/**
 * @brief 保护检测（每个控制周期调用）
 * @param self  实例
 * @param ia    A 相电流 A
 * @param ib    B 相电流 A
 * @param ic    C 相电流 A
 * @param vbus  母线电压 V
 * @param temp  温度 ℃（电机或功率级，取更高者）
 * @param speed 转速 rad/s（超速/欠速判定）
 * @param dt    控制周期 s
 * @return MCL_FAULT_NONE 或首个触发的故障码
 */
mcl_fault mcl_protection_check(mcl_protection *self, mcl_scalar ia, mcl_scalar ib, mcl_scalar ic,
                               mcl_scalar vbus, mcl_scalar temp, mcl_scalar speed, mcl_scalar dt);

/**
 * @brief 温度降额系数（用于电流限幅）
 * @param self 实例
 * @param temp 温度 ℃（电机或功率级，取更高者）
 * @return 1.0（正常）→ 0.0（接近关断温度），线性降额
 */
mcl_scalar mcl_protection_derate(mcl_protection *self, mcl_scalar temp);

#ifdef __cplusplus
}
#endif

#endif /* MCL_PROTECTION_H */
