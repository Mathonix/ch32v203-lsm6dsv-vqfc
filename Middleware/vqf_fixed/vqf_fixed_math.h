#ifndef VQF_FIXED_MATH_H
#define VQF_FIXED_MATH_H

#include <stdint.h>
#include <limits.h>
#include "vqf_fixed.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline int32_t vqf_sat_s32(int64_t x)
{
    if (x > INT32_MAX) {
        return INT32_MAX;
    }
    if (x < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)x;
}

static inline int32_t vqf_round_asr(int64_t x, unsigned s)
{
    if (s == 0u) {
        return vqf_sat_s32(x);
    }
    int64_t add = (int64_t)1 << (s - 1u);
    if (x < 0) {
        add = -add;
    }
    return vqf_sat_s32((x + add) >> s);
}

static inline int32_t vqf_mul_f30(int32_t a, int32_t b)
{
    return vqf_round_asr((int64_t)a * (int64_t)b, 30u);
}

static inline int32_t vqf_mul_shift(int32_t a, int32_t b, unsigned shift)
{
    return vqf_round_asr((int64_t)a * (int64_t)b, shift);
}

static inline int32_t vqf_clip_s32(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static inline int32_t vqf_iabs32(int32_t x)
{
    return (x < 0) ? -x : x;
}

uint32_t vqf_isqrt_u32(uint32_t x);
uint64_t vqf_isqrt_u64(uint64_t x);

/** Integer sqrt of F30 value → F30 (sqrt(x/2^30)*2^30). */
int32_t vqf_isqrt_f30(int32_t x_f30);

/**
 * Normalize F27 vector to F30 unit. Returns 0 if near-zero.
 * Uses isqrt on ||a||^2 (Q54) + division.
 */
int vqf_normalize_f27_to_f30(const int32_t a27[3], int32_t u30[3]);

#ifdef __cplusplus
}
#endif

#endif /* VQF_FIXED_MATH_H */
