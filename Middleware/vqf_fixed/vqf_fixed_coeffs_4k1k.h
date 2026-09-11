/**
 * Stub / alternate constants: gyr=4 kHz, acc=1 kHz (design default target).
 * Use when LSM6DSV GY ODR is independent HAODR 4 kHz; select via gyr_hz in init.
 * Regenerate via scripts/gen_vqf_fixed_coeffs.py.
 */
#ifndef VQF_FIXED_COEFFS_4K1K_H
#define VQF_FIXED_COEFFS_4K1K_H

#include <stdint.h>

#define VQF_C4K_DT_HALF_F30          ((int32_t)134218) /* 0.000125 */

/* rest gyr LP tau=0.5s @ 4kHz */
#define VQF_C4K_REST_GYR_B0          ((int32_t)134)
#define VQF_C4K_REST_GYR_B1          ((int32_t)268)
#define VQF_C4K_REST_GYR_B2          ((int32_t)134)
#define VQF_C4K_REST_GYR_A1          ((int32_t)-2146409906)
#define VQF_C4K_REST_GYR_A2          ((int32_t)1072668619)
#define VQF_C4K_REST_GYR_INIT_SAMPLES (2000u) /* 0.5/0.00025 */

/* Acc-side coeffs match 1k1k (acc still 1 kHz) — include that header for those. */

#endif /* VQF_FIXED_COEFFS_4K1K_H */
