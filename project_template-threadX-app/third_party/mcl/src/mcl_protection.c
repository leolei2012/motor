/**
 * @file    mcl_protection.c
 * @brief   mcl 电机控制库：保护检测实现
 */

#include "mcl_protection.h"
#include <string.h>

void mcl_protection_init(mcl_protection *self, const mcl_protection_limits *limits)
{
    if (self == NULL || limits == NULL)
    {
        return;
    }

    self->limits = *limits;
    memset(&self->status, 0, sizeof(self->status));
}

void mcl_protection_set_status(mcl_protection *self, const mcl_protection_status *status)
{
    if (self == NULL || status == NULL)
    {
        return;
    }
    self->status = *status;
}

mcl_fault mcl_protection_check(mcl_protection *self, mcl_scalar ia, mcl_scalar ib, mcl_scalar ic,
                               mcl_scalar vbus, mcl_scalar temp, mcl_scalar speed, mcl_scalar dt)
{
    if (self == NULL)
    {
        return MCL_FAULT_NONE;
    }

    /* 过流 */
    if ((self->limits.enabled & MCL_PROTECT_OVERCURRENT) &&
        (MCL_ABS(ia) > self->limits.overcurrent ||
         MCL_ABS(ib) > self->limits.overcurrent ||
         MCL_ABS(ic) > self->limits.overcurrent))
    {
        return MCL_FAULT_OVERCURRENT;
    }

    /* 过压 / 欠压 */
    if ((self->limits.enabled & MCL_PROTECT_OVERVOLTAGE) &&
        vbus > self->limits.overvoltage)
    {
        return MCL_FAULT_OVERVOLTAGE;
    }
    if ((self->limits.enabled & MCL_PROTECT_UNDERVOLTAGE) &&
        vbus < self->limits.undervoltage)
    {
        return MCL_FAULT_UNDERVOLTAGE;
    }

    /* 过温 */
    if ((self->limits.enabled & MCL_PROTECT_OVERTEMP) &&
        temp > self->limits.overtemp)
    {
        return MCL_FAULT_OVERTEMP;
    }

    if ((self->limits.enabled & MCL_PROTECT_ABS_OVERCURRENT) &&
        (MCL_ABS(ia) > self->limits.abs_overcurrent ||
         MCL_ABS(ib) > self->limits.abs_overcurrent ||
         MCL_ABS(ic) > self->limits.abs_overcurrent))
    {
        return MCL_FAULT_ABS_OVERCURRENT;
    }

    if ((self->limits.enabled & MCL_PROTECT_OVERTEMP_FET) &&
        self->status.temp_fet > self->limits.overtemp_fet)
    {
        return MCL_FAULT_OVERTEMP_FET;
    }
    if ((self->limits.enabled & MCL_PROTECT_OVERTEMP_MOTOR) &&
        self->status.temp_motor > self->limits.overtemp_motor)
    {
        return MCL_FAULT_OVERTEMP_MOTOR;
    }

    if ((self->limits.enabled & MCL_PROTECT_GATE_OV) &&
        self->status.gate_voltage > self->limits.gate_overvoltage)
    {
        return MCL_FAULT_GATE_OVERVOLTAGE;
    }
    if ((self->limits.enabled & MCL_PROTECT_GATE_UV) &&
        self->limits.gate_undervoltage > (mcl_scalar)0 &&
        self->status.gate_voltage < self->limits.gate_undervoltage)
    {
        return MCL_FAULT_GATE_UNDERVOLTAGE;
    }
    if ((self->limits.enabled & MCL_PROTECT_DRV) && self->status.drv_fault != 0u)
    {
        return MCL_FAULT_DRV;
    }

    if ((self->limits.enabled & MCL_PROTECT_SINCOS_LOW) &&
        self->limits.sincos_min > (mcl_scalar)0 &&
        self->status.sincos_amplitude < self->limits.sincos_min)
    {
        return MCL_FAULT_SINCOS_LOW;
    }
    if ((self->limits.enabled & MCL_PROTECT_SINCOS_HIGH) &&
        self->limits.sincos_max > (mcl_scalar)0 &&
        self->status.sincos_amplitude > self->limits.sincos_max)
    {
        return MCL_FAULT_SINCOS_HIGH;
    }

    if (self->limits.enabled & MCL_PROTECT_OFFSET)
    {
        if (MCL_ABS(self->status.current_offset[0]) > self->limits.offset_max)
        {
            return MCL_FAULT_OFFSET_1;
        }
        if (MCL_ABS(self->status.current_offset[1]) > self->limits.offset_max)
        {
            return MCL_FAULT_OFFSET_2;
        }
        if (MCL_ABS(self->status.current_offset[2]) > self->limits.offset_max)
        {
            return MCL_FAULT_OFFSET_3;
        }
    }

    if (self->limits.enabled & MCL_PROTECT_UNBALANCED)
    {
        mcl_scalar sum = MCL_ADD(MCL_ADD(ia, ib), ic);
        if (MCL_ABS(sum) > self->limits.unbalanced_max)
        {
            return MCL_FAULT_UNBALANCED;
        }
    }

    if ((self->limits.enabled & MCL_PROTECT_BRK) && self->status.brake_fault != 0u)
    {
        return MCL_FAULT_BRK;
    }
    if (self->limits.enabled & MCL_PROTECT_RESOLVER)
    {
        if (self->status.resolver_lot != 0u) { return MCL_FAULT_RESOLVER_LOT; }
        if (self->status.resolver_dos != 0u) { return MCL_FAULT_RESOLVER_DOS; }
        if (self->status.resolver_los != 0u) { return MCL_FAULT_RESOLVER_LOS; }
    }

    if ((self->limits.enabled & MCL_PROTECT_OVERSPEED) &&
        self->limits.overspeed > (mcl_scalar)0 &&
        MCL_ABS(speed) > self->limits.overspeed)
    {
        return MCL_FAULT_OVERSPEED;
    }
    if ((self->limits.enabled & MCL_PROTECT_UNDERSPEED) &&
        self->limits.underspeed > (mcl_scalar)0 &&
        MCL_ABS(speed) < self->limits.underspeed)
    {
        return MCL_FAULT_UNDERSPEED;
    }
    if ((self->limits.enabled & MCL_PROTECT_ABS_OVERSPEED) &&
        self->limits.abs_overspeed > (mcl_scalar)0 &&
        MCL_ABS(speed) > self->limits.abs_overspeed)
    {
        return MCL_FAULT_ABS_OVERSPEED;
    }

    return MCL_FAULT_NONE;
}

mcl_scalar mcl_protection_derate(mcl_protection *self, mcl_scalar temp)
{
    mcl_scalar range;

    /* 「系数 1.0 = 不降额」必须用 MCL_FROM_FLOAT(1.0f) 表达满幅：
     * 裸整数 (mcl_scalar)1 在 Q15/Q31 下是 1 LSB（≈0.00003 / ≈4.7e-7），
     * 会被误当成「几乎不降额到 0」，导致电流限幅直接失效。 */
    if (self == NULL)
    {
        return MCL_FROM_FLOAT(1.0f);
    }

    /* 过温保护关闭则不降额 */
    if (!(self->limits.enabled & MCL_PROTECT_OVERTEMP))
    {
        return MCL_FROM_FLOAT(1.0f);
    }

    if (temp <= self->limits.temp_derate_start)
    {
        return MCL_FROM_FLOAT(1.0f);
    }
    if (temp >= self->limits.overtemp)
    {
        return (mcl_scalar)0;
    }

    range = MCL_SUB(self->limits.overtemp, self->limits.temp_derate_start);
    if (range <= (mcl_scalar)0)
    {
        return (mcl_scalar)0;
    }

    /* 线性降额：(overtemp - temp) / (overtemp - temp_start)，结果 ∈ [0,1] */
    return MCL_DIV(MCL_SUB(self->limits.overtemp, temp), range);
}
