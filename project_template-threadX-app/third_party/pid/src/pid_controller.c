#include "pid_controller.h"

/**
 * @file   pid_controller.c
 * @brief  PI / PID 控制器实现 (16-bit 整数定点)
 *
 * 移植自 ST MCSDK pid_regulator：
 * - 比例/积分/微分增益均以 "分子 / 2^pow2" 分数形式表示
 * - 除法通过算术右移实现
 * - 积分项带上下限饱和防积分饱和 (anti-windup)
 * - 输出带上下限钳位
 */

/* ============================================================
   初始化
   ============================================================ */

/**
 * @brief  初始化 PID 控制器
 * @param  p PID 句柄
 *
 * 增益分子恢复到默认值, 积分项和上一次误差清零。
 */
void pid_init(pid_t *p)
{
    p->kp_gain               = p->def_kp_gain;
    p->ki_gain               = p->def_ki_gain;
    p->kd_gain               = p->def_kd_gain;
    p->integral_term         = 0;
    p->prev_process_var_error = 0;
}

/* ============================================================
   Kp 增益
   ============================================================ */

void pid_set_kp(pid_t *p, int16_t kp_gain)
{
    p->kp_gain = kp_gain;
}

int16_t pid_get_kp(const pid_t *p)
{
    return p->kp_gain;
}

int16_t pid_get_default_kp(const pid_t *p)
{
    return p->def_kp_gain;
}

uint16_t pid_get_kp_divisor(const pid_t *p)
{
    return p->kp_divisor;
}

uint16_t pid_get_kp_divisor_pow2(const pid_t *p)
{
    return p->kp_divisor_pow2;
}

void pid_set_kp_divisor_pow2(pid_t *p, uint16_t kp_divisor_pow2)
{
    p->kp_divisor_pow2 = kp_divisor_pow2;
    p->kp_divisor      = (uint16_t)(((uint32_t)1) << kp_divisor_pow2);
}

/* ============================================================
   Ki 增益
   ============================================================ */

void pid_set_ki(pid_t *p, int16_t ki_gain)
{
    p->ki_gain = ki_gain;
}

int16_t pid_get_ki(const pid_t *p)
{
    return p->ki_gain;
}

int16_t pid_get_default_ki(const pid_t *p)
{
    return p->def_ki_gain;
}

uint16_t pid_get_ki_divisor(const pid_t *p)
{
    return p->ki_divisor;
}

uint16_t pid_get_ki_divisor_pow2(const pid_t *p)
{
    return p->ki_divisor_pow2;
}

/**
 * @brief  设置 Ki 分母 (2 的幂次), 并自动更新积分项上下限
 * @param  p                PID 句柄
 * @param  ki_divisor_pow2  Ki 分母的 2 的幂次
 *
 * 积分项上下限被设为 ±INT16_MAX * Ki_divisor,
 * 即保证积分项除以分母后落在 int16_t 范围内。
 */
void pid_set_ki_divisor_pow2(pid_t *p, uint16_t ki_divisor_pow2)
{
    uint32_t ki_div = ((uint32_t)1) << ki_divisor_pow2;

    p->ki_divisor_pow2 = ki_divisor_pow2;
    p->ki_divisor      = (uint16_t)ki_div;
    pid_set_upper_integral_limit(p, (int32_t)INT16_MAX * (int32_t)ki_div);
    pid_set_lower_integral_limit(p, (int32_t)(-INT16_MAX) * (int32_t)ki_div);
}

/* ============================================================
   Kd 增益
   ============================================================ */

void pid_set_kd(pid_t *p, int16_t kd_gain)
{
    p->kd_gain = kd_gain;
}

int16_t pid_get_kd(const pid_t *p)
{
    return p->kd_gain;
}

uint16_t pid_get_kd_divisor(const pid_t *p)
{
    return p->kd_divisor;
}

uint16_t pid_get_kd_divisor_pow2(const pid_t *p)
{
    return p->kd_divisor_pow2;
}

void pid_set_kd_divisor_pow2(pid_t *p, uint16_t kd_divisor_pow2)
{
    p->kd_divisor_pow2 = kd_divisor_pow2;
    p->kd_divisor      = (uint16_t)(((uint32_t)1) << kd_divisor_pow2);
}

/* ============================================================
   积分项
   ============================================================ */

void pid_set_integral_term(pid_t *p, int32_t integral_term_value)
{
    p->integral_term = integral_term_value;
}

void pid_preload_integral_output(pid_t *p, int16_t output_value)
{
    int64_t ki_divisor;
    int64_t integral_term;

    if (p == NULL) return;

    ki_divisor = (int64_t)1 << p->ki_divisor_pow2;
    integral_term = (int64_t)output_value * ki_divisor;

    if (integral_term > p->upper_integral_limit)
    {
        integral_term = p->upper_integral_limit;
    }
    else if (integral_term < p->lower_integral_limit)
    {
        integral_term = p->lower_integral_limit;
    }

    p->integral_term = (int32_t)integral_term;
}

void pid_set_lower_integral_limit(pid_t *p, int32_t lower_limit)
{
    p->lower_integral_limit = lower_limit;
}

void pid_set_upper_integral_limit(pid_t *p, int32_t upper_limit)
{
    p->upper_integral_limit = upper_limit;
}

/* ============================================================
   微分项
   ============================================================ */

void pid_set_prev_error(pid_t *p, int32_t prev_process_var_error)
{
    p->prev_process_var_error = prev_process_var_error;
}

/* ============================================================
   控制器执行
   ============================================================ */

/**
 * @brief  PI 控制器计算
 * @param  p                PID 句柄
 * @param  process_var_error 当前过程误差 (参考值 - 当前值)
 * @retval PI 控制器输出 (int16_t)
 *
 * 公式: output = Kp * error + Ki * integral
 * 含积分项饱和 + 输出钳位 + 积分放电 (anti-windup)。
 */
int16_t pi_controller(pid_t *p, int32_t process_var_error)
{
    int32_t proportional_term;
    int32_t integral_term_inc;
    int32_t output_32;
    int32_t integral_sum_temp;
    int32_t discharge = 0;
    int64_t discharge_term;
    int64_t integral_after_discharge;
    int64_t ki_divisor;
    int16_t upper_output_limit = p->upper_output_limit;
    int16_t lower_output_limit = p->lower_output_limit;

    /* 比例项 */
    proportional_term = p->kp_gain * process_var_error;

    /* 积分项 (含溢出保护) */
    if (0 == p->ki_gain)
    {
        p->integral_term = 0;
    }
    else
    {
        integral_term_inc = p->ki_gain * process_var_error;
        integral_sum_temp = p->integral_term + integral_term_inc;

        if (integral_sum_temp < 0)
        {
            if (p->integral_term > 0)
            {
                if (integral_term_inc > 0)
                {
                    integral_sum_temp = INT32_MAX;
                }
            }
        }
        else
        {
            if (p->integral_term < 0)
            {
                if (integral_term_inc < 0)
                {
                    integral_sum_temp = -INT32_MAX;
                }
            }
        }

        if (integral_sum_temp > p->upper_integral_limit)
        {
            p->integral_term = p->upper_integral_limit;
        }
        else if (integral_sum_temp < p->lower_integral_limit)
        {
            p->integral_term = p->lower_integral_limit;
        }
        else
        {
            p->integral_term = integral_sum_temp;
        }
    }

    /* 合路: 比例项 + 积分项 (各除以其分母) */
    output_32 = (proportional_term >> p->kp_divisor_pow2)
              + (p->integral_term >> p->ki_divisor_pow2);

    /* 输出钳位 + 积分放电 */
    if (output_32 > upper_output_limit)
    {
        discharge  = upper_output_limit - output_32;
        output_32  = upper_output_limit;
    }
    else if (output_32 < lower_output_limit)
    {
        discharge  = lower_output_limit - output_32;
        output_32  = lower_output_limit;
    }

    ki_divisor = (int64_t)1 << p->ki_divisor_pow2;
    discharge_term = (int64_t)discharge * ki_divisor;
    integral_after_discharge = (int64_t)p->integral_term + discharge_term;

    if (integral_after_discharge > p->upper_integral_limit)
    {
        integral_after_discharge = p->upper_integral_limit;
    }
    else if (integral_after_discharge < p->lower_integral_limit)
    {
        integral_after_discharge = p->lower_integral_limit;
    }

    p->integral_term = (int32_t)integral_after_discharge;

    return (int16_t)output_32;
}

/**
 * @brief  PID 控制器计算
 * @param  p                PID 句柄
 * @param  process_var_error 当前过程误差 (参考值 - 当前值)
 * @retval PID 控制器输出 (int16_t)
 *
 * 在 PI 控制器基础上增加微分项:
 *   output = PI_output + Kd * (error - prev_error)
 * 若 Kd_gain == 0 则退化为纯 PI 控制器。
 */
int16_t pid_controller(pid_t *p, int32_t process_var_error)
{
    int32_t differential_term;
    int32_t delta_error;
    int32_t temp_output;

    if (0 == p->kd_gain)
    {
        temp_output = pi_controller(p, process_var_error);
    }
    else
    {
        delta_error       = process_var_error - p->prev_process_var_error;
        differential_term  = p->kd_gain * delta_error;

        differential_term >>= p->kd_divisor_pow2;

        p->prev_process_var_error = process_var_error;

        temp_output = pi_controller(p, process_var_error) + differential_term;

        if (temp_output > p->upper_output_limit)
        {
            temp_output = p->upper_output_limit;
        }
        else if (temp_output < p->lower_output_limit)
        {
            temp_output = p->lower_output_limit;
        }
    }

    return (int16_t)temp_output;
}
