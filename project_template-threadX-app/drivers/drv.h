#ifndef DRV_H
#define DRV_H

extern struct drv g_drv;

#include "platform.h"

#include "drv_ain_sensor.h"
#include "drv_output.h"
#include "drv_uart.h"
#include "drv_motor.h"

struct drv
{
    struct drv_ain_sensor *ain_sensor;
    struct drv_output     *output;
    struct drv_uart       *uart;
    struct drv_motor      *motor;
};

int drv_init(void);

#endif // DRV_H
