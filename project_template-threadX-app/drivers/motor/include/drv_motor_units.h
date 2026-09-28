#ifndef DRV_MOTOR_UNITS_H
#define DRV_MOTOR_UNITS_H

#include "mcl_types.h"

/* One source of truth for board/HAL/telemetry. V = I*R = W*flux = W*I*L. */
#define DRV_MOTOR_V_BASE       128.0f
#define DRV_MOTOR_I_BASE       32.0f
#define DRV_MOTOR_W_BASE       2048.0f
#define DRV_MOTOR_R_BASE       (DRV_MOTOR_V_BASE / DRV_MOTOR_I_BASE)
#define DRV_MOTOR_L_BASE       (DRV_MOTOR_R_BASE / DRV_MOTOR_W_BASE)
#define DRV_MOTOR_FLUX_BASE    (DRV_MOTOR_V_BASE / DRV_MOTOR_W_BASE)
#define DRV_MOTOR_TIME_BASE    (1.0f / DRV_MOTOR_W_BASE)
#define DRV_MOTOR_POLE_PAIRS   5u
#define DRV_MOTOR_RPM_BASE     (DRV_MOTOR_W_BASE * 9.5492966f / DRV_MOTOR_POLE_PAIRS)
#define DRV_MOTOR_ANGLE_BASE   6.28318530718f
#define DRV_MOTOR_TEMP_BASE    128.0f

#endif
