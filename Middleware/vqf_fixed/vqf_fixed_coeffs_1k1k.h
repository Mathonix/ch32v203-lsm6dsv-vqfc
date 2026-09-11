/**
 * PC-generated F30 constants: gyr=1 kHz, acc=1 kHz, tauAcc=3s, restTau=0.5s.
 * Regenerate via scripts/gen_vqf_fixed_coeffs.py — do not hand-edit.
 */
#ifndef VQF_FIXED_COEFFS_1K1K_H
#define VQF_FIXED_COEFFS_1K1K_H

#include <stdint.h>

/* dt/2 @ 1 kHz → F30 */
#define VQF_C1K_DT_HALF_F30          ((int32_t)536871)

/* acc LP tau=3s @ 1kHz */
#define VQF_C1K_ACC_LP_B0            ((int32_t)60)
#define VQF_C1K_ACC_LP_B1            ((int32_t)119)
#define VQF_C1K_ACC_LP_B2            ((int32_t)60)
#define VQF_C1K_ACC_LP_A1            ((int32_t)-2146767820)
#define VQF_C1K_ACC_LP_A2            ((int32_t)1073026235)
#define VQF_C1K_ACC_LP_INIT_SAMPLES  (3000u) /* round(tau/Ts) */

/* rest gyr/acc LP tau=0.5s @ 1kHz (same coeffs) */
#define VQF_C1K_REST_LP_B0           ((int32_t)2143)
#define VQF_C1K_REST_LP_B1           ((int32_t)4286)
#define VQF_C1K_REST_LP_B2           ((int32_t)2143)
#define VQF_C1K_REST_LP_A1           ((int32_t)-2143188686)
#define VQF_C1K_REST_LP_A2           ((int32_t)1069455435)
#define VQF_C1K_REST_LP_INIT_SAMPLES (500u)

/* thresholds / clips from design §6 */
#define VQF_C1K_BIAS_CLIP_F29        ((int32_t)18740330)   /* 2 deg/s */
#define VQF_C1K_BIAS_CLIP_F25        ((int32_t)1171271)    /* same physical, F25 */
#define VQF_C1K_REST_TH_GYR_F25      ((int32_t)1171271)
#define VQF_C1K_REST_TH_GYR2_Q50     (1371875755441ull)
#define VQF_C1K_REST_TH_ACC_F27      ((int32_t)6843200)    /* 0.5 m/s² in g */
#define VQF_C1K_REST_TH_ACC2_Q54     (46829386240000ull)
#define VQF_C1K_REST_MIN_SAMPLES     (1500u)               /* restMinT=1.5s @1kHz */

/* Bias Kalman: P F18, V F18, W U64/F8 */
#define VQF_C1K_BIAS_P0_F18          ((int32_t)655360000) /* 2500.0 */
#define VQF_C1K_BIAS_V_F18           ((int32_t)262)       /* 0.001 */
#define VQF_C1K_BIAS_W_REST_F8       (20738304ull)        /* 81009 */
#define VQF_C1K_BIAS_W_MOTION_F8     (2560025600ull)      /* 10000100 */
#define VQF_C1K_BIAS_W_VERT_F8       (25600256000000ull)  /* 100001000000 */

/* 1/Ts for motion e: Ts=0.001 → 1000, as F0 integer scale helper */
#define VQF_C1K_INV_ACC_TS           (1000)

#endif /* VQF_FIXED_COEFFS_1K1K_H */
