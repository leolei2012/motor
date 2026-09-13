/**
 * @file    pid_float.h
 * @brief   浮点 PI / PID 控制器 (对标 SguanFOC float PID)
 *
 * 与 ST MCSDK pid_t 不同，本模块使用纯 float 运算，适合与 float 控制变量配合。
 *
 * 公式:
 *   PI:  output = Kp * error + integral_term
 *        integral_term += Ki * error * dt  (受上下限钳位)
 *   PID: output += Kd * (error - prev_error) / dt
 *
 * Anti-windup: 输出钳位后自动放电积分项。
 *
 * @{
 */

#ifndef PID_FLOAT_H
#define PID_FLOAT_H

#include "platform.h"

/** @brief 浮点 PID 控制器句柄 */
typedef struct
{
    float kp;                       /**< 比例增益 (pu/pu) */
    float ki;                       /**< 积分增益 (pu/pu/s, 含 1/dt) */
    float kd;                       /**< 微分增益 (pu/pu*s) */
    float integral_term;            /**< 积分项累加值 */
    float upper_output_limit;       /**< 输出上限 */
    float lower_output_limit;       /**< 输出下限 */
    float upper_integral_limit;     /**< 积分项上限 */
    float lower_integral_limit;     /**< 积分项下限 */
    float prev_error;               /**< 上一拍误差 (微分项使用) */
} pid_float_t;

/* ============================================================
   初始化
   ============================================================ */

/**
 * @brief  初始化浮点 PID 控制器
 * @param  p PID 句柄
 *
 * 清零积分项和上一拍误差，保持 kp/ki/kd 和限值不变。
 */
void pid_float_init(pid_float_t *p);

/* ============================================================
   控制器执行
   ============================================================ */

/**
 * @brief  浮点 PI 控制器计算
 * @param  p     PID 句柄
 * @param  error 当前过程误差 (参考值 - 当前值)
 * @param  dt    距离上次调用的时间 (s)
 * @retval PI 控制器输出
 *
 * 公式: output = Kp * error + integral_term
 * 含积分项饱和 + 输出钳位 + 积分放电 (anti-windup)。
 */
float pi_float_controller(pid_float_t *p, float error, float dt);

/**
 * @brief  浮点 PID 控制器计算
 * @param  p     PID 句柄
 * @param  error 当前过程误差 (参考值 - 当前值)
 * @param  dt    距离上次调用的时间 (s)
 * @retval PID 控制器输出
 *
 * 在 PI 基础上增加微分项: output += Kd * (error - prev_error) / dt。
 * Kd == 0.0f 时退化为纯 PI。
 */
float pid_float_controller(pid_float_t *p, float error, float dt);

/* ============================================================
   辅助函数
   ============================================================ */

/**
 * @brief  预载积分项使初始输出接近目标值
 * @param  p            PID 句柄
 * @param  output_value 期望的初始输出值
 *
 * 典型用法: IF/TORQUE 启动时用开环 Vq 预载 PI 积分项,
 * 避免 PI 从零开始追赶。
 */
void pid_float_preload_integral_output(pid_float_t *p, float output_value);

/** @} */

#endif /* PID_FLOAT_H */
