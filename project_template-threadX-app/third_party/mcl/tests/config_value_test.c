#include "mcl_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Static initialization verifies these really are C constant expressions. */
static const mcl_config sample = {
    .phase_resistance = MCL_CONFIG_VALUE(0.475f, 8.0f),
    .bus_voltage = MCL_CONFIG_VALUE(24.0f, 64.0f),
    .avs_recovery_time = MCL_CONFIG_VALUE(0.05f, 1.0f)
};
static const mcl_scalar pos_limit = MCL_CONFIG_VALUE(128.0f, 64.0f);
static const mcl_scalar neg_limit = MCL_CONFIG_VALUE(-128.0f, 64.0f);
static const mcl_scalar negative = MCL_CONFIG_VALUE(-24.0f, 64.0f);
static const mcl_scalar near_one = MCL_CONFIG_VALUE(0.9999999999, 1.0);
int main(void)
{
#if defined(MCL_USE_Q15) || defined(MCL_USE_Q31)
    assert(fabsf(MCL_TO_FLOAT(sample.phase_resistance) - 0.475f/8.0f)<0.00004f);
    assert(fabsf(MCL_TO_FLOAT(sample.bus_voltage) - 24.0f/64.0f)<0.00004f);
    assert(pos_limit == MCL_FROM_FLOAT(1.0f));
    assert(neg_limit == MCL_FROM_FLOAT(-1.0f));
    assert(fabsf(MCL_TO_FLOAT(negative) + 24.0f/64.0f)<0.00004f);
    assert(near_one > 0);
#else
    assert(sample.phase_resistance == 0.475f);
    assert(sample.bus_voltage == 24.0f);
    assert(pos_limit == 128.0f && neg_limit == -128.0f);
    assert(negative == -24.0f && near_one == 1.0f);
#endif
    assert(fabsf(MCL_TO_FLOAT(sample.avs_recovery_time)-0.05f)<0.00004f);
    puts("Configuration constants: physical/normalized units, static initializer, saturation PASS");
    return 0;
}
