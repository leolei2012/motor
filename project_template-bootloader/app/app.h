#ifndef APP_H
#define APP_H

#include "platform.h"

#include "app_alarm_system.h"

extern struct app g_app;

struct app
{
    struct app_alarm_system *alarm_system;
};

#endif // APP_H
