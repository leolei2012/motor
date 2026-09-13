#include "app.h"

struct app g_app;

static struct app_alarm_system s_app_alarm_system;

/**
 * 应用层组合根：实例化 + 注入（零业务逻辑）。
 * 告警数据源来自 drivers 层（如 g_drv.ain_sensor 的某个物理量），
 * 用 app_alarm_system_register 注册后由主循环周期驱动 app_alarm_system_poll。
 */
int app_init(void)
{
    app_alarm_system_init(&s_app_alarm_system);
    g_app.alarm_system = &s_app_alarm_system;

    return 0;
}
