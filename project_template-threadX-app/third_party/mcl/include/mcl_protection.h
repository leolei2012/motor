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

/**
 * @brief 保护检测状态
 */
typedef struct
{
    mcl_protection_limits limits;   /**< 阈值 */
    mcl_protection_status status;   /**< 本拍宿主输入 */
} mcl_protection;

/**
 * @brief 初始化保护
 * @param self   实例
 * @param limits 阈值
 */
void mcl_protection_init(mcl_protection *self, const mcl_protection_limits *limits);

/** 写入本拍保护输入。未调用时输入为 0。 */
void mcl_protection_set_status(mcl_protection *self, const mcl_protection_status *status);

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
