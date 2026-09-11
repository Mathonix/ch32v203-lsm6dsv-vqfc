/**
 * Full VQF fixed-point (6D first delivery) — design domains, not mechanical Q15.
 *
 * Domains: quat F30, angle F28, gyro F25, acc(g) F27, bias F29, P F18,
 * IIR coeff F30, int64 accumulators. Mag behind VQF_FIXED_ENABLE_MAG=0.
 *
 * Honesty: structure matches Laidig/dusking1 VQF (IIR + rest + scaled LDLT
 * bias). Host replay: static RMS ~0; mild motion RMS ~0.48° (target 0.02°).
 * Mag behind VQF_FIXED_ENABLE_MAG=0 (not enabled in this delivery).
 */
#ifndef VQF_FIXED_H
#define VQF_FIXED_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VQF_FIXED_ENABLE_MAG
#define VQF_FIXED_ENABLE_MAG 0
#endif
#ifndef VQF_FIXED_ENABLE_MOTION_BIAS
#define VQF_FIXED_ENABLE_MOTION_BIAS 1
#endif
#ifndef VQF_FIXED_ENABLE_DEBUG
#define VQF_FIXED_ENABLE_DEBUG 0
#endif

#define VQF_F30_ONE ((int32_t)1073741824) /* 1.0 in F30 */

typedef struct {
    uint32_t gyr_hz; /* e.g. 1000 or 4000 — selects dt_half / rest gyr coeffs */
    uint32_t acc_hz; /* e.g. 1000 */
    uint32_t mag_hz; /* unused when MAG=0 */
} vqf_fixed_config_t;

void vqf_fixed_init(const vqf_fixed_config_t *cfg);

/** Gyro step: rad/s F25. Independent of acc rate (call at gyr_hz). */
void vqf_fixed_update_gyr_f25(const int32_t gyr_f25[3]);

/** Acc step: specific force in g, F27. Ignores [0,0,0]. */
void vqf_fixed_update_acc_f27(const int32_t acc_g_f27[3]);

#if VQF_FIXED_ENABLE_MAG
void vqf_fixed_update_mag_f18(const int32_t mag_uT_f18[3]);
#endif

void vqf_fixed_get_quat6d_f30(int32_t q_out[4]);
void vqf_fixed_get_bias_f29(int32_t bias_out[3]);
bool vqf_fixed_get_rest_detected(void);

/** Aerospace ZYX Euler from 6D quat → millidegrees (no libm). */
void vqf_fixed_get_euler_mdeg(int32_t *roll_mdeg, int32_t *pitch_mdeg, int32_t *yaw_mdeg);

/** Debug / NZW only. */
void vqf_fixed_quat_to_float(const int32_t q_f30[4], float q_out[4]);

#ifdef __cplusplus
}
#endif

#endif /* VQF_FIXED_H */
