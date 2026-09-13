#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "platform.h"

/**
 * @defgroup PID_Controller PID 控制器模块
 * @brief  经典 PI / PID 控制器 (16-bit 整数定点运算)
 *
 * 移植自 ST MCSDK pid_regulator，采用增益分子 + 2 的幂次分母的分数形式，
 * 通过算术右移替代除法以提升效率。
 *
 * 公式:
 *   PI:  r(t_k) = Kp * e(t_k) + Ki * sum_{j=0}^k e(t_j)
 *   PID: r(t_k) = Kp * e(t_k) + Ki * sum_{j=0}^k e(t_j) + Kd * (e(t_k) - e(t_{k-1}))
 *
 * 其中 Kp = kp_gain / 2^kp_div_pow2,
 *       Ki = ki_gain / 2^ki_div_pow2,
 *       Kd = kd_gain / 2^kd_div_pow2
 * @{
 */

/** @brief PID 控制器句柄 */
typedef struct
{
    int16_t  def_kp_gain;           /**< Kp 增益分子默认值 */
    int16_t  def_ki_gain;           /**< Ki 增益分子默认值 */
    int16_t  kp_gain;               /**< Kp 增益分子 */
    int16_t  ki_gain;               /**< Ki 增益分子 */
    int32_t  integral_term;         /**< 积分项 (除以 Ki 分母前的原始值) */
    int32_t  upper_integral_limit;  /**< 积分项上限 */
    int32_t  lower_integral_limit;  /**< 积分项下限 */
    int16_t  upper_output_limit;    /**< 输出上限 */
    int16_t  lower_output_limit;    /**< 输出下限 */
    uint16_t kp_divisor;            /**< Kp 分母 (= 2^kp_divisor_pow2) */
    uint16_t ki_divisor;            /**< Ki 分母 (= 2^ki_divisor_pow2) */
    uint16_t kp_divisor_pow2;       /**< Kp 分母的 2 的幂次 (右移位数) */
    uint16_t ki_divisor_pow2;       /**< Ki 分母的 2 的幂次 (右移位数) */
    int16_t  def_kd_gain;           /**< Kd 增益分子默认值 */
    int16_t  kd_gain;               /**< Kd 增益分子 */
    uint16_t kd_divisor;            /**< Kd 分母 (= 2^kd_divisor_pow2) */
    uint16_t kd_divisor_pow2;       /**< Kd 分母的 2 的幂次 (右移位数) */
    int32_t  prev_process_var_error; /**< 上一次过程误差 (微分项使用) */
} pid_t;

/* ============================================================
   初始化
   ============================================================ */

void pid_init(pid_t *p);

/* ============================================================
   Kp 增益
   ============================================================ */

void     pid_set_kp(pid_t *p, int16_t kp_gain);
int16_t  pid_get_kp(const pid_t *p);
int16_t  pid_get_default_kp(const pid_t *p);
uint16_t pid_get_kp_divisor(const pid_t *p);
uint16_t pid_get_kp_divisor_pow2(const pid_t *p);
void     pid_set_kp_divisor_pow2(pid_t *p, uint16_t kp_divisor_pow2);

/* ============================================================
   Ki 增益
   ============================================================ */

void     pid_set_ki(pid_t *p, int16_t ki_gain);
int16_t  pid_get_ki(const pid_t *p);
int16_t  pid_get_default_ki(const pid_t *p);
uint16_t pid_get_ki_divisor(const pid_t *p);
uint16_t pid_get_ki_divisor_pow2(const pid_t *p);
void     pid_set_ki_divisor_pow2(pid_t *p, uint16_t ki_divisor_pow2);

/* ============================================================
   Kd 增益
   ============================================================ */

void     pid_set_kd(pid_t *p, int16_t kd_gain);
int16_t  pid_get_kd(const pid_t *p);
uint16_t pid_get_kd_divisor(const pid_t *p);
uint16_t pid_get_kd_divisor_pow2(const pid_t *p);
void     pid_set_kd_divisor_pow2(pid_t *p, uint16_t kd_divisor_pow2);

/* ============================================================
   积分项
   ============================================================ */

void pid_set_integral_term(pid_t *p, int32_t integral_term_value);
void pid_preload_integral_output(pid_t *p, int16_t output_value);
void pid_set_lower_integral_limit(pid_t *p, int32_t lower_limit);
void pid_set_upper_integral_limit(pid_t *p, int32_t upper_limit);

/* ============================================================
   输出限制 (inline)
   ============================================================ */

static inline void pid_set_lower_output_limit(pid_t *p, int16_t lower_limit)
{
    p->lower_output_limit = lower_limit;
}

static inline void pid_set_upper_output_limit(pid_t *p, int16_t upper_limit)
{
    p->upper_output_limit = upper_limit;
}

/* ============================================================
   微分项
   ============================================================ */

void pid_set_prev_error(pid_t *p, int32_t prev_process_var_error);

/* ============================================================
   控制器执行
   ============================================================ */

int16_t pi_controller(pid_t *p, int32_t process_var_error);
int16_t pid_controller(pid_t *p, int32_t process_var_error);

/** @} */

#endif /* PID_CONTROLLER_H */
