/**
 * @file    pid_float.c
 * @brief   浮点 PI / PID 控制器实现 (对标 SguanFOC float PID)
 *
 * 公式:
 *   PI:  output = Kp * error + integral_term
 *        integral_term += Ki * error * dt
 *
 * Anti-windup: 输出钳位后积分项自动放电, 防止积分饱和。
 */

#include "pid_float.h"

/* ============================================================
   初始化
   ============================================================ */

/**
 * @brief  初始化浮点 PID 控制器
 * @param  p PID 句柄
 */
void pid_float_init(pid_float_t *p)
{
    if (p == NULL) return;
    p->integral_term = 0.0f;
    p->prev_error    = 0.0f;
}

/* ============================================================
   PI 控制器
   ============================================================ */

/**
 * @brief  浮点 PI 控制器计算
 * @param  p     PID 句柄
 * @param  error 当前过程误差
 * @param  dt    距离上次调用的时间 (s)
 * @retval PI 控制器输出
 */
float pi_float_controller(pid_float_t *p, float error, float dt)
{
    float p_term;
    float i_increment;
    float output;
    float discharge;

    if (p == NULL) return 0.0f;

    /* 比例项 */
    p_term = p->kp * error;

    /* 积分项 (含 anti-windup 钳位) */
    if (p->ki != 0.0f && dt > 0.0f)
    {
        i_increment = p->ki * error * dt;
        p->integral_term += i_increment;

        if (p->integral_term > p->upper_integral_limit)
            p->integral_term = p->upper_integral_limit;
        else if (p->integral_term < p->lower_integral_limit)
            p->integral_term = p->lower_integral_limit;
    }

    /* 合路 */
    output = p_term + p->integral_term;

    /* 输出钳位 + 积分放电 */
    if (output > p->upper_output_limit)
    {
        discharge = p->upper_output_limit - output;
        output    = p->upper_output_limit;
        p->integral_term += discharge;
        if (p->integral_term > p->upper_integral_limit)
            p->integral_term = p->upper_integral_limit;
        else if (p->integral_term < p->lower_integral_limit)
            p->integral_term = p->lower_integral_limit;
    }
    else if (output < p->lower_output_limit)
    {
        discharge = p->lower_output_limit - output;
        output    = p->lower_output_limit;
        p->integral_term += discharge;
        if (p->integral_term > p->upper_integral_limit)
            p->integral_term = p->upper_integral_limit;
        else if (p->integral_term < p->lower_integral_limit)
            p->integral_term = p->lower_integral_limit;
    }

    return output;
}

/* ============================================================
   PID 控制器
   ============================================================ */

/**
 * @brief  浮点 PID 控制器计算
 * @param  p     PID 句柄
 * @param  error 当前过程误差
 * @param  dt    距离上次调用的时间 (s)
 * @retval PID 控制器输出
 */
float pid_float_controller(pid_float_t *p, float error, float dt)
{
    float pi_out;
    float d_term;
    float output;

    if (p == NULL) return 0.0f;

    /* PI 部分 */
    pi_out = pi_float_controller(p, error, dt);

    /* 微分部分 */
    if (p->kd != 0.0f && dt > 0.0f)
    {
        d_term = p->kd * (error - p->prev_error) / dt;
        p->prev_error = error;
        output = pi_out + d_term;

        /* 输出钳位 */
        if (output > p->upper_output_limit)
            output = p->upper_output_limit;
        else if (output < p->lower_output_limit)
            output = p->lower_output_limit;

        return output;
    }

    return pi_out;
}

/* ============================================================
   预载
   ============================================================ */

/**
 * @brief  预载积分项使初始输出接近目标值
 * @param  p            PID 句柄
 * @param  output_value 期望的初始输出值
 *
 * 将 integral_term 设为 output_value, 使第一拍输出 = Kp*error + output_value。
 * 常用于 IF/TORQUE 启动时用开环 Vq 预载 PI, 避免从零追赶。
 */
void pid_float_preload_integral_output(pid_float_t *p, float output_value)
{
    if (p == NULL) return;

    p->integral_term = output_value;

    if (p->integral_term > p->upper_integral_limit)
        p->integral_term = p->upper_integral_limit;
    else if (p->integral_term < p->lower_integral_limit)
        p->integral_term = p->lower_integral_limit;
}
