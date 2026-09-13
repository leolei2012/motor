#ifndef APP_H
#define APP_H

#include "platform.h"

#include "app_alarm_system.h"
#include "app_led.h"

extern struct app g_app;

struct app
{
    struct app_alarm_system *alarm_system;
    struct app_led          *led;
};

int app_init(void);

#endif // APP_H
