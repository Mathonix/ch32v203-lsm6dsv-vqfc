#include "vqf_fixed_math.h"
#include "flash_zw.h"

FLASH_ZW_CODE uint32_t vqf_isqrt_u32(uint32_t x)
{
    if (x == 0u) {
        return 0u;
    }
    uint32_t s = 1u << ((32u - (uint32_t)__builtin_clz(x) + 1u) >> 1);
    for (int i = 0; i < 6; i++) {
        s = (s + x / s) >> 1;
    }
    if (s > 0u && (uint64_t)s * (uint64_t)s > x) {
        s--;
    }
    return s;
}

FLASH_ZW_CODE uint64_t vqf_isqrt_u64(uint64_t x)
{
    if (x == 0ull) {
        return 0ull;
    }
    uint64_t s = x;
    if (x > 1ull) {
        unsigned lz = (unsigned)__builtin_clzll(x);
        s = 1ull << ((64u - lz + 1u) >> 1);
    }
    for (int i = 0; i < 8; i++) {
        s = (s + x / s) >> 1;
    }
    if (s > 0ull && s > x / s) {
        s--;
    }
    return s;
}

FLASH_ZW_CODE int32_t vqf_isqrt_f30(int32_t x_f30)
{
    if (x_f30 <= 0) {
        return 0;
    }
    /* sqrt(x / 2^30) * 2^30 = 2^15 * sqrt(x) when x is the F30 integer */
    uint32_t r = vqf_isqrt_u32((uint32_t)x_f30);
    return (int32_t)(r << 15);
}

FLASH_ZW_CODE int vqf_normalize_f27_to_f30(const int32_t a27[3], int32_t u30[3])
{
    uint64_t n54 = (uint64_t)((int64_t)a27[0] * a27[0]) +
                   (uint64_t)((int64_t)a27[1] * a27[1]) +
                   (uint64_t)((int64_t)a27[2] * a27[2]);
    if (n54 < 16ull) {
        return 0;
    }
    /* ||a|| in F27 = isqrt(n54); want u = a * 2^30 / ||a||
     * = (a << 30) / isqrt(n54). isqrt(n54) is F27. */
    uint64_t n = vqf_isqrt_u64(n54);
    if (n == 0ull) {
        return 0;
    }
    u30[0] = vqf_sat_s32((((int64_t)a27[0] << 30) / (int64_t)n));
    u30[1] = vqf_sat_s32((((int64_t)a27[1] << 30) / (int64_t)n));
    u30[2] = vqf_sat_s32((((int64_t)a27[2] << 30) / (int64_t)n));
    return 1;
}
