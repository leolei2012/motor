#include "mcl.h"
#include "drv_motor_config.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    mcl motor;
    mcl_config cfg, copy;
    mcl_hal_ops hal = {0};
    mcl_observer_smo observer;
    mcl_observer_smo_params op, observed;
    mcl_motor_parameters params, original, readback;
    mcl_diagnostics diag;
    mcl_observer_smo_diagnostics obs_diag;
    mcl_protection_status status = {0}, status_copy;
    mcl_scalar offsets[3];
    uint32_t frequency;
    drv_motor_config_default(&cfg, &op);
    assert(mcl_init(&motor, &cfg, &hal, NULL, &mcl_observer_smo_ops, &observer, &op) == MCL_OK);
    assert(mcl_get_motor_parameters(&motor, &original) == MCL_OK);
    params = original;
    params.phase_resistance = MCL_CONFIG_VALUE(0.5f, DRV_MOTOR_R_BASE);
    params.phase_inductance = MCL_CONFIG_VALUE(0.0009f, DRV_MOTOR_L_BASE);
    params.bemf_const = MCL_CONFIG_VALUE(0.008f, DRV_MOTOR_FLUX_BASE);
    assert(mcl_set_motor_parameters(&motor, &params) == MCL_OK);
    assert(mcl_get_config(&motor, &copy) == MCL_OK);
    assert(copy.phase_resistance == params.phase_resistance);
    assert(copy.phase_inductance == params.phase_inductance);
    assert(mcl_observer_smo_get_params(&observer, &observed) == MCL_OK);
    assert(observed.resistance == params.phase_resistance);
    assert(observed.inductance == params.phase_inductance);
    assert(observed.ld == MCL_SUB(params.phase_inductance, params.ld_lq_diff));
    assert(observed.flux == params.bemf_const);
    /* White-box assertions verify the setter synchronizes all algorithm caches. */
    assert(motor.foc.phase_resistance == params.phase_resistance);
    assert(motor.foc.mtpa_fw.lq == params.phase_inductance);
    assert(motor.foc.mtpa_fw.ld == observed.ld);
    assert(motor.mtpa_fw.lambda == params.bemf_const);
    observed.gain = 0;
    assert(mcl_observer_smo_set_params(&observer, &observed) == MCL_ERR_PARAM);
    assert(mcl_observer_smo_get_params(&observer, &observed) == MCL_OK);
    assert(observed.gain == op.gain);
    original = params;
    params.phase_inductance = params.ld_lq_diff;
    assert(mcl_set_motor_parameters(&motor, &params) == MCL_ERR_PARAM);
    assert(mcl_get_motor_parameters(&motor, &readback) == MCL_OK);
    assert(readback.phase_inductance == original.phase_inductance);
    assert(mcl_observer_smo_get_params(&observer, &observed) == MCL_OK);
    assert(observed.inductance == original.phase_inductance);

    assert(mcl_get_control_frequency(&motor, &frequency) == MCL_OK && frequency == 16000);
    assert(mcl_get_current_offsets(&motor, offsets) == MCL_OK && offsets[0] == 0);
    status.drv_fault = 1;
    assert(mcl_set_protection_status(&motor, &status) == MCL_OK);
    status.drv_fault = 0;
    assert(mcl_get_protection_status(&motor, &status_copy) == MCL_OK && status_copy.drv_fault == 1);
    assert(mcl_set_speed(&motor, MCL_CONFIG_VALUE(800.0f, DRV_MOTOR_RPM_BASE)) == MCL_OK);
    assert(mcl_get_diagnostics(&motor, &diag) == MCL_OK);
    assert(diag.speed_ref_rpm == MCL_CONFIG_VALUE(800.0f, DRV_MOTOR_RPM_BASE));
    diag.speed_ref_rpm = 0;
    assert(mcl_get_diagnostics(&motor, &diag) == MCL_OK && diag.speed_ref_rpm != 0);
    assert(mcl_observer_smo_get_diagnostics(&observer, &obs_diag) == MCL_OK && obs_diag.phase == 0);
    /* Single-value accessors preserve command units and physical-second timers. */
    assert(mcl_get_ctrl_mode(&motor) == MCL_CTRL_SPEED);
    assert(mcl_get_speed_ref_rpm(&motor) == MCL_CONFIG_VALUE(800.0f, DRV_MOTOR_RPM_BASE));
    assert(mcl_get_iq_now(&motor) == diag.iq_now);
    assert(mcl_get_dt(&motor) > 0);
    assert(mcl_get_ol_timer(&motor) == 0.0f);
    assert(mcl_get_tick_count(&motor) == 0u);
    assert(mcl_start(&motor) == MCL_OK);
    assert(mcl_set_motor_parameters(&motor, &original) == MCL_ERR_STATE);
    assert(mcl_get_diagnostics(&motor, &diag) == MCL_OK && diag.state == MCL_STATE_RUN);
    assert(mcl_get_diagnostics(NULL, &diag) == MCL_ERR_PARAM);
    assert(mcl_get_current_offsets(&motor, NULL) == MCL_ERR_PARAM);
    assert(mcl_set_motor_parameters(NULL, &original) == MCL_ERR_PARAM);
    puts("Host API: copy semantics, validation, stopped-only setter and cache synchronization PASS");
    return 0;
}
