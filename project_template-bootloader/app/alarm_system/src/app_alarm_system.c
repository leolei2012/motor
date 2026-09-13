#include "app_alarm_system.h"

/**
 * @brief 初始化告警系统
 */
void app_alarm_system_init(struct app_alarm_system *self)
{
    uint8_t i;

    if (self == NULL)
    {
        return;
    }

    memset(self, 0, sizeof(*self));

    for (i = 0; i < APP_ALARM_SYSTEM_ID_NUM; i++)
    {
        alarm_init(&self->alarms[i]);
        self->alarms[i].alarm_id = i;
    }
}

/**
 * @brief 注册告警
 */
int8_t app_alarm_system_register(struct app_alarm_system *self, const struct alarm_config_t *config)
{
    if (self == NULL || config == NULL)
    {
        return -1;
    }

    if (config->alarm_id >= APP_ALARM_SYSTEM_ID_NUM)
    {
        return -1;
    }

    return alarm_config(&self->alarms[config->alarm_id], config);
}

int8_t app_alarm_system_enable_alarm(struct app_alarm_system *self, uint8_t alarm_id)
{
    if (self == NULL || alarm_id >= APP_ALARM_SYSTEM_ID_NUM)
    {
        return -1;
    }

    alarm_enable(&self->alarms[alarm_id]);

    return 0;
}

int8_t app_alarm_system_disable_alarm(struct app_alarm_system *self, uint8_t alarm_id)
{
    if (self == NULL || alarm_id >= APP_ALARM_SYSTEM_ID_NUM)
    {
        return -1;
    }

    alarm_disable(&self->alarms[alarm_id]);

    return 0;
}

int8_t app_alarm_system_clear_alarm(struct app_alarm_system *self, uint8_t alarm_id)
{
    if (self == NULL || alarm_id >= APP_ALARM_SYSTEM_ID_NUM)
    {
        return -1;
    }

    alarm_clear(&self->alarms[alarm_id]);

    return 0;
}

void app_alarm_system_clear_all(struct app_alarm_system *self)
{
    uint8_t i;

    if (self == NULL)
    {
        return;
    }

    for (i = 0; i < APP_ALARM_SYSTEM_ID_NUM; i++)
    {
        alarm_clear(&self->alarms[i]);
    }

    self->active_alarm_count = 0;
}

enum alarm_status_t app_alarm_system_get_status(const struct app_alarm_system *self, uint8_t alarm_id)
{
    if (self == NULL || alarm_id >= APP_ALARM_SYSTEM_ID_NUM)
    {
        return ALARM_STATUS_DISABLED;
    }

    return alarm_get_status(&self->alarms[alarm_id]);
}

uint8_t app_alarm_system_has_active_alarms(const struct app_alarm_system *self)
{
    if (self == NULL)
    {
        return 0;
    }

    return (self->active_alarm_count > 0) ? 1 : 0;
}

uint8_t app_alarm_system_get_active_count(const struct app_alarm_system *self)
{
    if (self == NULL)
    {
        return 0;
    }

    return self->active_alarm_count;
}

struct alarm *app_alarm_system_get_alarm(struct app_alarm_system *self, uint8_t alarm_id)
{
    if (self == NULL || alarm_id >= APP_ALARM_SYSTEM_ID_NUM)
    {
        return NULL;
    }

    return &self->alarms[alarm_id];
}

uint32_t app_alarm_system_get_trigger_count(const struct app_alarm_system *self, uint8_t alarm_id)
{
    if (self == NULL || alarm_id >= APP_ALARM_SYSTEM_ID_NUM)
    {
        return 0;
    }

    return self->alarms[alarm_id].trigger_count;
}

void app_alarm_system_poll(struct app_alarm_system *self)
{
    uint8_t active_count;
    uint8_t i;

    if (self == NULL)
    {
        return;
    }

    active_count = 0;

    for (i = 0; i < APP_ALARM_SYSTEM_ID_NUM; i++)
    {
        alarm_run(&self->alarms[i]);

        if (self->alarms[i].status == ALARM_STATUS_ACTIVE)
        {
            active_count++;
        }
    }

    self->active_alarm_count = active_count;
}
