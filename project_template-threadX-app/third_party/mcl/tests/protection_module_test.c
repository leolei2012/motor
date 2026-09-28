#include "mcl_protection.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Links against protection.c alone: no facade, HAL, motor or observer. */
int main(void)
{
    mcl_protection p;
    mcl_protection_limits limits = {0};
    mcl_protection_sample s = {0};
    mcl_fault_info info;
    mcl_protection_status status = {0};
    unsigned n;
    limits.enabled = MCL_PROTECT_OVERCURRENT | MCL_PROTECT_OVERVOLTAGE | MCL_PROTECT_DRV;
    limits.overcurrent = MCL_FROM_FLOAT(0.5f);
    limits.overvoltage = MCL_FROM_FLOAT(0.8f);
    memset(&p, 0xA5, sizeof(p));
    mcl_protection_init(&p, &limits);
    assert(mcl_protection_get_fault(&p) == MCL_FAULT_NONE);
    assert(p.info.fault == MCL_FAULT_NONE && p.recovery_elapsed_s == 0.0f);

    s.vbus = MCL_FROM_FLOAT(0.8f);
    assert(mcl_protection_update(&p, &s) == MCL_FAULT_NONE); /* equality is permitted */
    s.vbus = MCL_FROM_FLOAT(0.9f);
    s.current = MCL_FROM_FLOAT(-0.2f);
    s.speed = MCL_FROM_FLOAT(0.3f);
    s.temp = MCL_FROM_FLOAT(0.4f);
    s.tick = 17;
    assert(mcl_protection_update(&p, &s) == MCL_FAULT_OVERVOLTAGE);
    assert(mcl_protection_get_fault_info(&p, &info) == MCL_OK);
    assert(info.voltage == s.vbus && info.current == s.current);
    assert(info.speed == s.speed && info.temp == s.temp && info.tick == 17);
    s.tick = 99;
    s.vbus = MCL_FROM_FLOAT(0.4f);
    assert(mcl_protection_update(&p, &s) == MCL_FAULT_OVERVOLTAGE); /* latched */
    assert(mcl_protection_assert(&p, MCL_FAULT_DRV, &s) == MCL_OK);
    assert(p.info.tick == 17 && p.info.fault == MCL_FAULT_OVERVOLTAGE);
    for (n = 0; n < 100; ++n) { assert(!mcl_protection_advance(&p, 1.0f)); }
    mcl_protection_clear(&p);
    assert(p.fault == MCL_FAULT_NONE && p.info.tick == 17); /* retain history */
    assert(mcl_protection_update(&p, &s) == MCL_FAULT_NONE);

    /* Configuration updates never discard a fault; timer is physical seconds. */
    mcl_protection_configure(&p, &limits, 0.5f);
    status.drv_fault = 1;
    mcl_protection_set_status(&p, &status);
    assert(mcl_protection_update(&p, &s) == MCL_FAULT_DRV);
    assert(!mcl_protection_advance(&p, 0.25f));
    assert(!mcl_protection_advance(&p, NAN));
    assert(!mcl_protection_advance(&p, -1.0f));
    mcl_protection_configure(&p, &limits, 0.5f);
    assert(mcl_protection_assert(&p, MCL_FAULT_BRK, &s) == MCL_OK);
    assert(p.recovery_elapsed_s == 0.25f && p.fault == MCL_FAULT_DRV);
    assert(mcl_protection_advance(&p, 0.25f));
    assert(p.fault == MCL_FAULT_NONE && p.info.fault == MCL_FAULT_DRV);
    /* Automatic clear is acknowledgement only: persistent input trips again. */
    assert(mcl_protection_update(&p, &s) == MCL_FAULT_DRV);
    assert(mcl_protection_assert(&p, MCL_FAULT_NONE, &s) == MCL_ERR_PARAM);
    assert(mcl_protection_get_fault_info(&p, NULL) == MCL_ERR_PARAM);
    puts("Standalone protection: thresholds, first-event latch, snapshot, clear, physical-time recovery PASS");
    return 0;
}
