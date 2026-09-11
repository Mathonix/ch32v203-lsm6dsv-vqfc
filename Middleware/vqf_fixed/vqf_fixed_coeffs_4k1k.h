/**
 * PC-generated F30 constants: gyr=4 kHz, acc=1 kHz (design default).
 * Acc-side / Kalman / rest-acc coeffs live in vqf_fixed_coeffs_1k1k.h.
 * Regenerate via scripts/gen_vqf_fixed_coeffs.py — do not hand-edit.
 * Selected at runtime when vqf_fixed_config_t.gyr_hz >= 3500.
 */
#ifndef VQF_FIXED_COEFFS_4K1K_H
#define VQF_FIXED_COEFFS_4K1K_H

#include <stdint.h>

#define VQF_C4K_DT_HALF_F30          ((int32_t)134218) /* 0.000125 */

/* rest gyr LP tau=0.5s @ 4 kHz */
#define VQF_C4K_REST_GYR_B0          ((int32_t)134)
#define VQF_C4K_REST_GYR_B1          ((int32_t)268)
#define VQF_C4K_REST_GYR_B2          ((int32_t)134)
#define VQF_C4K_REST_GYR_A1          ((int32_t)-2146409906)
#define VQF_C4K_REST_GYR_A2          ((int32_t)1072668619)
#define VQF_C4K_REST_GYR_INIT_SAMPLES (2000u) /* 0.5/0.00025 */

/* Mag path coeffs (100 Hz) — unused while VQF_FIXED_ENABLE_MAG=0 */
#define VQF_C4K_MAG_LP_B0            ((int32_t)17816143)
#define VQF_C4K_MAG_LP_B1            ((int32_t)35632287)
#define VQF_C4K_MAG_LP_B2            ((int32_t)17816143)
#define VQF_C4K_MAG_LP_A1            ((int32_t)-1722274864)
#define VQF_C4K_MAG_LP_A2            ((int32_t)719797614)
#define VQF_C4K_K_MAG_F30            ((int32_t)1192384)
#define VQF_C4K_K_MAG_REF_F30        ((int32_t)536737)

#endif /* VQF_FIXED_COEFFS_4K1K_H */
