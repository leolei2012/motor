#ifndef HAL_CORDIC_H
#define HAL_CORDIC_H

#include "platform.h"

/**
 * @defgroup HAL_CORDIC CORDIC 硬件抽象层
 * @brief  硬件 CORDIC 协处理器接口，芯片相关实现位于 hal/src/hal_cordic.c
 * @{
 */

/**
 * @brief 定义此宏表示当前芯片拥有硬件 CORDIC 协处理器。
 *        上层宏调度层 (foc_cordic.h) 据此选择硬件路径或软件后备路径。
 */
#define HAL_CORDIC_ENABLED

/**
 * @brief 使能 CORDIC 时钟 (AHB1)
 */
void hal_cordic_init(void);

/**
 * @brief  CORDIC 余弦/正弦计算 (Q1.15)
 * @note   写入角度，同时返回 cos 和 sin 分量。
 * @param  angle  角度 (Q1.15)
 * @param  p_cos  输出余弦值 (Q1.15)
 * @param  p_sin  输出正弦值 (Q1.15)
 */
void hal_cordic_sincos(int16_t angle, int16_t *p_cos, int16_t *p_sin);

/**
 * @brief  CORDIC 平方根 (Q1.31)
 * @param  x 输入值 (≥0)
 * @retval sqrt(x) 整数近似值；x≤0 时返回 0
 */
int32_t hal_cordic_sqrt(int32_t x);

/**
 * @brief  CORDIC 矢量幅值 (Q1.15)
 * @param  alpha alpha 分量 (Q1.15)
 * @param  beta  beta 分量 (Q1.15)
 * @retval sqrt(alpha^2 + beta^2) (Q1.15)
 */
int16_t hal_cordic_modulus(int16_t alpha, int16_t beta);

/**
 * @brief  CORDIC 反正切 (Q1.31 → Q1.15)
 * @param  alpha alpha 分量 (Q1.31)
 * @param  beta  beta 分量 (Q1.31)
 * @retval atan2(beta, alpha) 电角度 (Q1.15)
 */
int16_t hal_cordic_phase(int32_t alpha, int32_t beta);

/** @} */

#endif /* HAL_CORDIC_H */
