#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "platform.h"

#include "mcl.h"
#include "mcl_observer_ortega.h"

/**
 * @file drv_motor.h
 * @brief 电机驱动：在 drivers 层实例化 mcl 算法库，实现 mcl_hal_ops 硬件注入，
 *        并封装 mcl 面门 API 为设备级接口。
 *
 * 层次依赖：app → drv_motor(mcl) → hal_tim1 / hal_adc1 / hal_adc2
 * mcl 实例由本模块持有（static），不占用堆内存。
 */

struct drv_motor
{
    mcl motor;                    /**< mcl 电机对象（算法核心） */
    mcl_observer_ortega observer; /**< Ortega 磁链观测器实例（λ²−|λ|² 幅值反馈，角度自动收敛） */
    float r_meas;               /**< 启动实测相电阻 Ω（0=未测） */
    float l_meas;               /**< 启动实测相电感 H（0=未测） */
    float ia_now;               /**< 最近一拍 A 相电流 A（已减零漂，供调试观测） */
    float ib_now;               /**< 最近一拍 B 相电流 A */
    float ic_now;               /**< 最近一拍 C 相电流 A */
    float cap_pre_est;          /**< 切换捕获：拖动最后一拍帧角 rad */
    float cap_pre_lam;          /**< 切换捕获：拖动最后一拍观测器输出角 rad */
    float cap_post_est;         /**< 切换捕获：闭环第一拍帧角 rad（PLL 输出） */
    float cap_post_lam;         /**< 切换捕获：闭环第一拍观测器输出角 rad */
    float cap_post_spd;         /**< 切换捕获：闭环第一拍 PLL 速度 rad/s */
    float cap_pll_last;         /**< 切换捕获：闭环第一拍 PLL.last_phase（上一拍输入=拖动末拍 seed 前观测器输出） */
    float cap_in_prev1;         /**< 切换捕获：拖动最后第二拍 PLL 输入角 rad */
    float cap_in_prev2;         /**< 切换捕获：拖动最后第三拍 PLL 输入角 rad */
    uint8_t cap_valid;          /**< 切换捕获：1=已捕获一次（诊断用） */
    uint8_t start_step;         /**< 启动进度（1=使能驱动 2=TIM1 3=零漂校准 4=MOE 5=R/L 实测 6=已 start；诊断用） */

    /* ---- 切换后逐拍采集（对数间隔 1,2,4,...,8192 拍，定位闭环崩溃） ---- */
    uint32_t cap2_switch_count; /**< 拖动→闭环切换次数（每轮 +1） */
    uint32_t cap2_ticks;        /**< 最近一轮闭环存活拍数（重开环/停机时锁存，0=尚未完成一轮） */
    uint8_t  cap2_valid_n;      /**< 最近一轮已采集的偏移点数（0~14） */
    float    cap2_min_spd;      /**< 最近一轮闭环最低速度 rad/s */
    float    cap2_max_iq;       /**< 最近一轮闭环最大 |iq| A */
    float    cap2_frame[14];    /**< 各偏移拍帧角 rad（PLL 输出） */
    float    cap2_obs[14];      /**< 各偏移拍观测器输出角 rad */
    float    cap2_spd[14];      /**< 各偏移拍 PLL 速度 rad/s */
    float    cap2_va[14];       /**< 各偏移拍 Vα（上周期指令，母线归一化） */
    float    cap2_vb[14];       /**< 各偏移拍 Vβ（上周期指令，母线归一化） */
    float    cap2_x1[14];       /**< 各偏移拍观测器状态 x1（定子磁链 α） */
    float    cap2_x2[14];       /**< 各偏移拍观测器状态 x2（定子磁链 β） */
    float    cap2_lam[14];      /**< 各偏移拍观测器磁链幅值 lambda_est */

    /* ---- 锁存影子（重开环瞬间在 ISR 内整体拷贝，Modbus 读影子保证整轮一致） ---- */
    uint32_t cap2_s_switch_count;
    uint32_t cap2_s_ticks;
    uint8_t  cap2_s_valid_n;
    float    cap2_s_min_spd;
    float    cap2_s_max_iq;
    float    cap2_s_frame[14];
    float    cap2_s_obs[14];
    float    cap2_s_spd[14];
    float    cap2_s_va[14];
    float    cap2_s_vb[14];
    float    cap2_s_x1[14];
    float    cap2_s_x2[14];
    float    cap2_s_lam[14];
};

/**
 * @brief 初始化电机驱动（构造 mcl 配置 + 注入 HAL + 绑定观测器）
 * @param self 电机驱动对象
 * @return 0=成功，-1=失败
 */
int drv_motor_init(struct drv_motor *self);

/**
 * @brief 启动电机（FOC 无感模式）
 * @param self 电机驱动对象
 * @return 0=成功，-1=失败
 */
int drv_motor_start(struct drv_motor *self);

/**
 * @brief 停止电机
 * @param self 电机驱动对象
 * @return 0=成功，-1=失败
 */
int drv_motor_stop(struct drv_motor *self);

/**
 * @brief 设置 q 轴电流（电流环指令）
 * @param self   电机驱动对象
 * @param iq_ref q 轴电流参考 A
 * @return 0=成功，-1=失败
 */
int drv_motor_set_current(struct drv_motor *self, float iq_ref);

/**
 * @brief 设置速度（速度环指令）
 * @param self      电机驱动对象
 * @param speed_rpm 速度参考 rpm
 * @return 0=成功，-1=失败
 */
int drv_motor_set_speed(struct drv_motor *self, float speed_rpm);

/**
 * @brief 开环 VF 旋转电压矢量（全开环，电流环不闭合）
 * @param self      电机驱动对象
 * @param voltage   电压矢量幅值（标幺 [-1,1]，1.0=满母线）
 * @param speed_rpm 目标机械转速 rpm（相位斜坡斜率）
 * @return 0=成功，-1=失败
 */
int drv_motor_set_openloop_vf(struct drv_motor *self, float voltage, float speed_rpm);

/**
 * @brief 开环 IF 旋转电流矢量（相位开环、电流环闭环）
 * @param self      电机驱动对象
 * @param current   电流矢量幅值 A（q 轴电流参考，建议 ≤ 额定电流）
 * @param speed_rpm 目标机械转速 rpm（相位斜坡斜率）
 * @return 0=成功，-1=失败
 */
int drv_motor_set_openloop_if(struct drv_motor *self, float current, float speed_rpm);

/**
 * @brief 电流环控制节拍（在 ADC/PWM 更新中断里以固定频率调用）
 * @param self 电机驱动对象
 */
void drv_motor_control_isr(struct drv_motor *self);

/**
 * @brief 电流零漂重新校准（电机停转、PWM 无输出时调用）
 * @param self 电机驱动对象
 * @return 0=成功，-1=失败
 */
int drv_motor_calibrate_offset(struct drv_motor *self);

/**
 * @brief 启动实测相电阻/相电感（直流 α 注入 + 电压脉冲法）
 *
 * 前置条件：TIM1 已启动（ADC 注入组在采样）、MOE 已使能、
 * mcl 未 start（状态 IDLE，控制节拍不抢占 PWM 输出）。
 * 结果写入 self->r_meas / self->l_meas，并回填观测器/电流环参数。
 *
 * @param self 电机驱动对象
 * @return 0=成功，-1=失败
 */
int drv_motor_measure_rl(struct drv_motor *self);

/**
 * @brief 读取电机遥测（观测变量快照，供调试/监控层查询）
 * @param self 电机驱动对象
 * @param out  遥测（输出，mcl_telemetry 结构体）
 * @return 0=成功，-1=失败
 */
int drv_motor_get_telemetry(struct drv_motor *self, mcl_telemetry *out);

#endif /* DRV_MOTOR_H */
