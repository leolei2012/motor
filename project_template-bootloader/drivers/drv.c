#include "drv.h"

#include "hal.h"

struct drv g_drv;

static struct drv_ain_sensor s_ain_sensor;

int drv_init(void)
{
    drv_ain_sensor_init(&s_ain_sensor);
    g_drv.ain_sensor = &s_ain_sensor;

    return 0;
}
