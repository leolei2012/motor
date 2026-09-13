#ifndef ALARM_H
#define ALARM_H

#include "platform.h"

/// 告警阈值比较类型
enum alarm_threshold_t
{
    GREATER_OR_EQUAL_THAN_THRESHOLD = 0, /// 大于等于阈值
    GREATER_THAN_THRESHOLD,              /// 大于阈值
    LESS_OR_EQUAL_THAN_THRESHOLD,        /// 小于等于阈值
    LESS_THAN_THRESHOLD,                 /// 小于阈值
    EQUAL_TO_THRESHOLD,                  /// 等于阈值
    NOT_EQUAL_TO_THRESHOLD               /// 不等于阈值
};

/// 告警状态
enum alarm_status_t
{
    ALARM_STATUS_NORMAL = 0, /// 正常
    ALARM_STATUS_ACTIVE,     /// 告警中
    ALARM_STATUS_DISABLED    /// 告警已禁用
};

/// 告警事件类型
enum alarm_event_t
{
    ALARM_EVENT_TRIGGERED = 0, /// 告警触发
    ALARM_EVENT_RECOVERED      /// 告警恢复

};

/// 告警对象结构体
struct alarm
{
    uint8_t alarm_id;           /// 告警ID，范围0-255
    enum alarm_status_t status; /// 当前告警状态
    uint8_t enabled;            /// 告警使能标志 (0=禁用, 1=使能)

    enum alarm_threshold_t threshold_type; /// 阈值类型
    uint16_t threshold_value;              /// 阈值数值
    uint16_t recovery_diff_value;          /// 回差值（恢复阈值）

    uint16_t *data_source; /// 数据源指针（指向需要监控的变量）

    uint8_t duration;     /// 持续多长时间才报告警，单位为time_base的倍数
    uint8_t elapsed_time; /// 已经过的时间，单位为time_base的倍数

    uint8_t recovery_time;         /// 恢复时间，单位为time_base的倍数
    uint8_t recovery_elapsed_time; /// 已经过的恢复时间，单位为time_base的倍数

    void (*callback)(uint8_t alarm_id, enum alarm_event_t event);


    uint16_t trigger_count;     /// 告警触发次数统计
#if 0
    uint32_t last_trigger_time; /// 最后一次触发时间戳（可选）
#endif
};

/// 告警配置结构体
struct alarm_config_t
{
    uint8_t alarm_id;                      /// 告警ID
    enum alarm_threshold_t threshold_type; /// 阈值类型
    uint16_t threshold_value;              /// 阈值数值
    uint16_t recovery_diff_value;          /// 回差值
    uint16_t *data_source;                 /// 数据源指针
    uint8_t duration;                      /// 触发延时时间
    uint8_t recovery_time;                 /// 恢复延时时间

#if 1
    void (*callback)(uint8_t alarm_id, enum alarm_event_t event); /// 回调函数
#endif
};

/**
 * @brief 初始化告警对象
 * @param self 告警对象指针
 */
void alarm_init(struct alarm *self);

/**
 * @brief 配置告警参数
 * @param self 告警对象指针
 * @param config 告警配置参数
 * @return 0=成功, -1=失败
 */
int8_t alarm_config(struct alarm *self, const struct alarm_config_t *config);

/**
 * @brief 运行告警检测
 * @param self 告警对象指针
 * @note 需要周期性调用此函数进行告警检测
 */
void alarm_run(struct alarm *self);

/**
 * @brief 使能告警
 * @param self 告警对象指针
 */
void alarm_enable(struct alarm *self);

/**
 * @brief 禁用告警
 * @param self 告警对象指针
 */
void alarm_disable(struct alarm *self);

/**
 * @brief 清除告警状态
 * @param self 告警对象指针
 */
void alarm_clear(struct alarm *self);

/**
 * @brief 获取告警状态
 * @param self 告警对象指针
 * @return 告警状态
 */
enum alarm_status_t alarm_get_status(const struct alarm *self);

/**
 * @brief 判断告警条件是否满足
 * @param self 告警对象指针
 * @param current_value 当前监控值
 * @return 1=满足, 0=不满足
 */
static uint8_t alarm_check_condition(const struct alarm *self, uint16_t current_value);

/**
 * @brief 判断恢复条件是否满足
 * @param self 告警对象指针
 * @param current_value 当前监控值
 * @return 1=满足, 0=不满足
 */
static uint8_t alarm_check_recovery(const struct alarm *self, uint16_t current_value);

#endif // ALARM_H