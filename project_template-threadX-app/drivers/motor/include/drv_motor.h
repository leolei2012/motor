#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "platform.h"

#include "mcl.h"
#include "mcl_observer_flux.h"

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
    mcl_observer_flux observer;   /**< 磁链观测器实例 */
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

#endif /* DRV_MOTOR_H */
