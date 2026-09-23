/* Physical-unit regression: voltage[k-1], measured current[k], 16 kHz.
 * Exercise the board's R/L/flux and both directions, without trusting PLL speed. */
#include "mcl_observer_smo.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static float wrap(float x)
{
    while (x > 3.14159265f) x -= 6.2831853f;
    while (x < -3.14159265f) x += 6.2831853f;
    return x;
}

static int run(float rpm, float iq)
{
    const float dt = 1.0f / 16000.0f, r = 0.475f, l = 0.0008f, flux = 0.00717f;
    const float w = rpm * 5.0f * 6.2831853f / 60.0f;
    mcl_observer_smo obs;
    mcl_observer_smo_params p = {r, l, flux, 10.0f, 6.28f, 0.5f};
    float theta = 0.4f, ia = -iq * sinf(theta), ib = iq * cosf(theta);
    float sum = 0.0f, max_err = 0.0f, est = 0.0f, speed = 0.0f;
    int n;
    memset(&obs, 0, sizeof(obs));
    mcl_observer_smo_ops.init(&obs, &p);
    for (n = 0; n < 24000; ++n)
    {
        float next = wrap(theta + w * dt);
        float na = -iq * sinf(next), nb = iq * cosf(next);
        /* Exact discrete plant equation used by the observer's Euler predictor. */
        float va = r * ia + l * (na - ia) / dt - w * flux * sinf(theta);
        float vb = r * ib + l * (nb - ib) / dt + w * flux * cosf(theta);
        /* Open-loop hint for 0.5s, then observer must sustain itself. */
        mcl_observer_smo_set_seed_omega(&obs, n < 8000 ? w : 0.0f);
        mcl_observer_smo_ops.update(&obs, va, vb, na, nb, dt, &est, &speed);
        theta = next; ia = na; ib = nb; if (n == 7999) printf("  release: omega=%.2f filter=%.6f err=%.2f\n", speed, obs.filter_step, wrap(est-theta)*57.29578f);
        if (!isfinite(est) || !isfinite(speed)) return 1;
        if (n >= 16000)
        {
            float err = fabsf(wrap(est - theta));
            sum += err;
            if (err > max_err) max_err = err;
        }
    }
    printf("rpm=%7.0f iq=%4.1f mean=%6.2fdeg max=%6.2fdeg speed=%8.2f (expected %.2f)\n",
           rpm, iq, sum / 8000.0f * 57.29578f, max_err * 57.29578f, speed, w);
    return sum / 8000.0f > 0.175f || max_err > 0.35f || fabsf(speed - w) > fabsf(w) * 0.10f;
}

int main(void)
{
    const float speeds[] = {100.0f, 300.0f, 800.0f, 2000.0f, -300.0f, -800.0f, -2000.0f};
    int i, failures = 0;
    for (i = 0; i < (int)(sizeof(speeds) / sizeof(speeds[0])); ++i)
        failures += run(speeds[i], speeds[i] > 0 ? 3.0f : -3.0f);
    printf("SMO regression: %d failures\n", failures);
    return failures ? 1 : 0;
}
