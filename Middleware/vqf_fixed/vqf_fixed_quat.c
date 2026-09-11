#include "vqf_fixed_quat.h"
#include "vqf_fixed_math.h"
#include "flash_zw.h"

FLASH_ZW_CODE void vqf_quat_mul_f30(const int32_t a[4], const int32_t b[4], int32_t o[4])
{
    int64_t t0 = (int64_t)a[0] * b[0] - (int64_t)a[1] * b[1] -
                 (int64_t)a[2] * b[2] - (int64_t)a[3] * b[3];
    int64_t t1 = (int64_t)a[0] * b[1] + (int64_t)a[1] * b[0] +
                 (int64_t)a[2] * b[3] - (int64_t)a[3] * b[2];
    int64_t t2 = (int64_t)a[0] * b[2] - (int64_t)a[1] * b[3] +
                 (int64_t)a[2] * b[0] + (int64_t)a[3] * b[1];
    int64_t t3 = (int64_t)a[0] * b[3] + (int64_t)a[1] * b[2] -
                 (int64_t)a[2] * b[1] + (int64_t)a[3] * b[0];
    o[0] = vqf_round_asr(t0, 30u);
    o[1] = vqf_round_asr(t1, 30u);
    o[2] = vqf_round_asr(t2, 30u);
    o[3] = vqf_round_asr(t3, 30u);
}

FLASH_ZW_CODE void vqf_quat_normalize_f30(int32_t q[4])
{
    int64_t n2 = (int64_t)q[0] * q[0] + (int64_t)q[1] * q[1] +
                 (int64_t)q[2] * q[2] + (int64_t)q[3] * q[3];
    if (n2 <= 0) {
        q[0] = VQF_F30_ONE;
        q[1] = q[2] = q[3] = 0;
        return;
    }
    /* ||q||_F30 ≈ isqrt(n2) with n2 in Q60 → isqrt(n2)>>0 is Q30 */
    uint64_t n = vqf_isqrt_u64((uint64_t)n2);
    if (n == 0ull) {
        return;
    }
    q[0] = vqf_sat_s32(((int64_t)q[0] << 30) / (int64_t)n);
    q[1] = vqf_sat_s32(((int64_t)q[1] << 30) / (int64_t)n);
    q[2] = vqf_sat_s32(((int64_t)q[2] << 30) / (int64_t)n);
    q[3] = vqf_sat_s32(((int64_t)q[3] << 30) / (int64_t)n);
}

FLASH_ZW_CODE void vqf_quat_normalize_f30_if_needed(int32_t q[4])
{
    int64_t n2 = (int64_t)q[0] * q[0] + (int64_t)q[1] * q[1] +
                 (int64_t)q[2] * q[2] + (int64_t)q[3] * q[3];
    /* 1.0^2 in Q60 = 2^60; allow ~2% drift before renormalize (integer only) */
    const int64_t one2 = (int64_t)1 << 60;
    const int64_t lo = one2 - (one2 >> 5); /* ~0.968 */
    const int64_t hi = one2 + (one2 >> 5);
    if (n2 < lo || n2 > hi) {
        vqf_quat_normalize_f30(q);
    }
}

FLASH_ZW_CODE void vqf_quat_to_rotmat_f30(const int32_t q[4], int32_t R[9])
{
    int32_t w = q[0], x = q[1], y = q[2], z = q[3];
    int32_t xx = vqf_mul_f30(x, x), yy = vqf_mul_f30(y, y), zz = vqf_mul_f30(z, z);
    int32_t xy = vqf_mul_f30(x, y), xz = vqf_mul_f30(x, z), yz = vqf_mul_f30(y, z);
    int32_t wx = vqf_mul_f30(w, x), wy = vqf_mul_f30(w, y), wz = vqf_mul_f30(w, z);
    R[0] = VQF_F30_ONE - ((yy + zz) << 1);
    R[1] = (xy - wz) << 1;
    R[2] = (xz + wy) << 1;
    R[3] = (xy + wz) << 1;
    R[4] = VQF_F30_ONE - ((xx + zz) << 1);
    R[5] = (yz - wx) << 1;
    R[6] = (xz - wy) << 1;
    R[7] = (yz + wx) << 1;
    R[8] = VQF_F30_ONE - ((xx + yy) << 1);
}

FLASH_ZW_CODE void vqf_rotmat_vec(const int32_t R[9], const int32_t v[3], int32_t out[3])
{
    out[0] = vqf_round_asr((int64_t)R[0] * v[0] + (int64_t)R[1] * v[1] + (int64_t)R[2] * v[2], 30u);
    out[1] = vqf_round_asr((int64_t)R[3] * v[0] + (int64_t)R[4] * v[1] + (int64_t)R[5] * v[2], 30u);
    out[2] = vqf_round_asr((int64_t)R[6] * v[0] + (int64_t)R[7] * v[1] + (int64_t)R[8] * v[2], 30u);
}

FLASH_ZW_CODE void vqf_quat_rotate_vec(const int32_t q[4], const int32_t v[3], int32_t out[3])
{
    int32_t R[9];
    vqf_quat_to_rotmat_f30(q, R);
    vqf_rotmat_vec(R, v, out);
}

FLASH_ZW_CODE void vqf_quat_delta_from_v_f30(const int32_t v30[3], int32_t dq[4])
{
    int64_t s60 = (int64_t)v30[0] * v30[0] + (int64_t)v30[1] * v30[1] +
                  (int64_t)v30[2] * v30[2];
    int32_t x2 = vqf_round_asr(s60, 30u);
    int32_t x4 = vqf_mul_f30(x2, x2);
    int32_t c = VQF_F30_ONE - (x2 >> 1) + (int32_t)((int64_t)x4 / 24);
    int32_t sinc = VQF_F30_ONE - (int32_t)((int64_t)x2 / 6) + (int32_t)((int64_t)x4 / 120);
    dq[0] = c;
    dq[1] = vqf_mul_f30(sinc, v30[0]);
    dq[2] = vqf_mul_f30(sinc, v30[1]);
    dq[3] = vqf_mul_f30(sinc, v30[2]);
}
