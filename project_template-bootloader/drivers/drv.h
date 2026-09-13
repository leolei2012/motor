#ifndef DRV_H
#define DRV_H

extern struct drv g_drv;

#include "platform.h"

#include "drv_ain_sensor.h"

struct drv
{
    struct drv_ain_sensor *ain_sensor;
};

int drv_init(void);

#endif // DRV_H
