#include "alarm.h"
#include <string.h>

static uint8_t alarm_check_condition(const struct alarm *self, uint16_t current_value)
{
    if (self == NULL)
    {
        return 0;
    }

    switch (self->threshold_type)
    {
    case GREATER_OR_EQUAL_THAN_THRESHOLD:
        return (current_value >= self->threshold_value) ? 1 : 0;

    case GREATER_THAN_THRESHOLD:
        return (current_value > self->threshold_value) ? 1 : 0;

    case LESS_OR_EQUAL_THAN_THRESHOLD:
        return (current_value <= self->threshold_value) ? 1 : 0;

    case LESS_THAN_THRESHOLD:
        return (current_value < self->threshold_value) ? 1 : 0;

    case EQUAL_TO_THRESHOLD:
        return (current_value == self->threshold_value) ? 1 : 0;

    case NOT_EQUAL_TO_THRESHOLD:
        return (current_value != self->threshold_value) ? 1 : 0;

    default:
        return 0;
    }
}

static uint8_t alarm_check_recovery(const struct alarm *self, uint16_t current_value)
{
    uint16_t recovery_threshold;

    if (self == NULL)
    {
        return 0;
    }

    recovery_threshold = 0;

    /// 根据告警类型计算恢复阈值（带回差）
    switch (self->threshold_type)
    {
    case GREATER_OR_EQUAL_THAN_THRESHOLD:
    case GREATER_THAN_THRESHOLD:
        /// 对于大于类型的告警，恢复需要小于(阈值-回差值)
        recovery_threshold = self->threshold_value - self->recovery_diff_value;
        return (current_value < recovery_threshold) ? 1 : 0;

    case LESS_OR_EQUAL_THAN_THRESHOLD:
    case LESS_THAN_THRESHOLD:
        /// 对于小于类型的告警，恢复需要大于(阈值+回差值)
        recovery_threshold = self->threshold_value + self->recovery_diff_value;
        return (current_value > recovery_threshold) ? 1 : 0;

    case EQUAL_TO_THRESHOLD:
        /// 对于等于类型的告警，恢复需要不等于阈值
        return (current_value != self->threshold_value) ? 1 : 0;

    case NOT_EQUAL_TO_THRESHOLD:
        /// 对于不等于类型的告警，恢复需要等于阈值
        return (current_value == self->threshold_value) ? 1 : 0;

    default:
        return 0;
    }
}

/**
 * @brief 初始化告警对象
 */
void alarm_init(struct alarm *self)
{
    if (self == NULL)
    {
        return;
    }

    memset(self, 0, sizeof(*self));
    self->status = ALARM_STATUS_NORMAL;
    self->enabled = 0;
}

int8_t alarm_config(struct alarm *self, const struct alarm_config_t *config)
{
    if (self == NULL || config == NULL)
    {
        return -1;
    }

    if (config->data_source == NULL)
    {
        return -1;
    }

    self->alarm_id = config->alarm_id;
    self->threshold_type = config->threshold_type;
    self->threshold_value = config->threshold_value;
    self->recovery_diff_value = config->recovery_diff_value;
    self->data_source = config->data_source;
    self->duration = config->duration;
    self->recovery_time = config->recovery_time;
    self->callback = config->callback; /// 保存回调函数

    /// 清除运行状态
    self->elapsed_time = 0;
    self->recovery_elapsed_time = 0;
    self->status = ALARM_STATUS_NORMAL;
    self->enabled = 1; /// 配置后自动使能

    return 0;
}

void alarm_run(struct alarm *self)
{
    uint16_t current_value;

    if (self == NULL || !self->enabled || self->data_source == NULL)
    {
        return;
    }

    current_value = *(self->data_source);

    /// 根据当前状态执行不同的逻辑
    switch (self->status)
    {
    case ALARM_STATUS_NORMAL:
    {
        /// 检查是否满足告警条件
        if (alarm_check_condition(self, current_value))
        {
            self->elapsed_time++;

            /// 持续满足条件达到设定时长，触发告警
            if (self->elapsed_time >= self->duration)
            {
                self->status = ALARM_STATUS_ACTIVE;
                self->elapsed_time = 0;
                self->recovery_elapsed_time = 0;


                self->trigger_count++;


                if (self->callback != NULL)
                {
                    self->callback(self->alarm_id, ALARM_EVENT_TRIGGERED);
                }


            }
        }
        else
        {
            /// 条件不满足，重置计时
            self->elapsed_time = 0;
        }
        break;
    }

    case ALARM_STATUS_ACTIVE:
    {
        /// 检查是否满足恢复条件
        if (alarm_check_recovery(self, current_value))
        {
            self->recovery_elapsed_time++;

            /// 持续满足恢复条件达到设定时长，告警恢复
            if (self->recovery_elapsed_time >= self->recovery_time)
            {
                self->status = ALARM_STATUS_NORMAL;
                self->elapsed_time = 0;
                self->recovery_elapsed_time = 0;

                if (self->callback != NULL)
                {
                    self->callback(self->alarm_id, ALARM_EVENT_RECOVERED);
                }


            }
        }
        else
        {
            /// 恢复条件不满足，重置恢复计时
            self->recovery_elapsed_time = 0;
        }
        break;
    }

    case ALARM_STATUS_DISABLED:
    default:
        /// 禁用状态，不做处理
        break;
    }
}

void alarm_enable(struct alarm *self)
{
    if (self == NULL)
    {
        return;
    }

    self->enabled = 1;
    if (self->status == ALARM_STATUS_DISABLED)
    {
        self->status = ALARM_STATUS_NORMAL;
    }
}

void alarm_disable(struct alarm *self)
{
    if (self == NULL)
    {
        return;
    }

    self->enabled = 0;
    self->status = ALARM_STATUS_DISABLED;
    self->elapsed_time = 0;
    self->recovery_elapsed_time = 0;
}

void alarm_clear(struct alarm *self)
{
    if (self == NULL)
    {
        return;
    }

    self->status = ALARM_STATUS_NORMAL;
    self->elapsed_time = 0;
    self->recovery_elapsed_time = 0;
}

enum alarm_status_t alarm_get_status(const struct alarm *self)
{
    if (self == NULL)
    {
        return ALARM_STATUS_DISABLED;
    }

    return self->status;
}