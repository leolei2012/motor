#ifndef APP_ALARM_SYSTEM_H
#define APP_ALARM_SYSTEM_H

#include "platform.h"
#include "alarm.h"

/**
 * 告警 ID 枚举（模板示例，按需替换为实际业务告警）
 */
enum app_alarm_system_id_t
{
    APP_ALARM_SYSTEM_ID_OVER_TEMPERATURE, /**< 示例告警：温度超上限 */
    APP_ALARM_SYSTEM_ID_UNDER_VOLTAGE,    /**< 示例告警：电压欠压 */

    APP_ALARM_SYSTEM_ID_NUM,
};

struct app_alarm_system
{
    struct alarm alarms[APP_ALARM_SYSTEM_ID_NUM]; /**< 告警对象数组 */
    uint8_t active_alarm_count;                   /**< 当前活动的告警数量 */
};

/**
 * @brief 初始化告警系统
 * @param self 告警系统对象指针
 */
void app_alarm_system_init(struct app_alarm_system *self);

/**
 * @brief 注册告警
 * @param self 告警系统对象指针
 * @param config 告警配置参数
 * @return 0=成功, -1=失败
 */
int8_t app_alarm_system_register(struct app_alarm_system *self, const struct alarm_config_t *config);

/**
 * @brief 使能指定告警
 * @param self 告警系统对象指针
 * @param alarm_id 告警ID
 * @return 0=成功, -1=失败
 */
int8_t app_alarm_system_enable_alarm(struct app_alarm_system *self, uint8_t alarm_id);

/**
 * @brief 禁用指定告警
 * @param self 告警系统对象指针
 * @param alarm_id 告警ID
 * @return 0=成功, -1=失败
 */
int8_t app_alarm_system_disable_alarm(struct app_alarm_system *self, uint8_t alarm_id);

/**
 * @brief 清除指定告警状态
 * @param self 告警系统对象指针
 * @param alarm_id 告警ID
 * @return 0=成功, -1=失败
 */
int8_t app_alarm_system_clear_alarm(struct app_alarm_system *self, uint8_t alarm_id);

/**
 * @brief 清除所有告警状态
 * @param self 告警系统对象指针
 */
void app_alarm_system_clear_all(struct app_alarm_system *self);

/**
 * @brief 获取指定告警的状态
 * @param self 告警系统对象指针
 * @param alarm_id 告警ID
 * @return 告警状态
 */
enum alarm_status_t app_alarm_system_get_status(const struct app_alarm_system *self, uint8_t alarm_id);

/**
 * @brief 检查是否有活动告警
 * @param self 告警系统对象指针
 * @return 1=有活动告警, 0=无活动告警
 */
uint8_t app_alarm_system_has_active_alarms(const struct app_alarm_system *self);

/**
 * @brief 获取活动告警数量
 * @param self 告警系统对象指针
 * @return 活动告警数量
 */
uint8_t app_alarm_system_get_active_count(const struct app_alarm_system *self);

/**
 * @brief 获取告警对象指针
 * @param self 告警系统对象指针
 * @param alarm_id 告警ID
 * @return 告警对象指针，失败返回NULL
 */
struct alarm *app_alarm_system_get_alarm(struct app_alarm_system *self, uint8_t alarm_id);

/**
 * @brief 获取告警触发次数
 * @param self 告警系统对象指针
 * @param alarm_id 告警ID
 * @return 告警触发次数
 */
uint32_t app_alarm_system_get_trigger_count(const struct app_alarm_system *self, uint8_t alarm_id);

#define APP_ALARM_SYSTEM_TASK_PERIOD 1000
void app_alarm_system_poll(struct app_alarm_system *self);

#endif // APP_ALARM_SYSTEM_H
