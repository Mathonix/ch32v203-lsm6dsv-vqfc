/**
 * Full VQF fixed-point 6D state machine (design §9–§12, §14).
 * Mag deferred (VQF_FIXED_ENABLE_MAG=0).
 */
#include "vqf_fixed.h"
#include "vqf_fixed_math.h"
#include "vqf_fixed_quat.h"
#include "vqf_fixed_biquad.h"
#include "vqf_fixed_ldlt.h"
#include "vqf_fixed_coeffs_1k1k.h"
#include "vqf_fixed_coeffs_4k1k.h"
#include "flash_zw.h"
#include "flash_nzw.h"

#include <string.h>

typedef struct {
    int32_t gyrQuat[4];
    int32_t accQuat[4];
    int32_t bias_f29[3];
    int32_t biasP_f18[9];

    int32_t lastAccLp_f27[3];
    vqf_biquad_state_t accLp[3];
    vqf_biquad_state_t motionRlp[9];
    vqf_biquad_state_t motionBiasLp[2];

    int32_t restGyrLp_f25[3];
    int32_t restAccLp_f27[3];
    vqf_biquad_state_t restGyrLp[3];
    vqf_biquad_state_t restAccLp[3];
    uint64_t restGyrDev_q50;
    uint64_t restAccDev_q54;
    uint32_t rest_samples;
    uint8_t restDetected;

    uint32_t gyr_norm_counter;
    uint32_t acc_norm_counter;

    int32_t dt_half_f30;
    vqf_biquad_coeff_f30_t coeff_acc_lp;
    vqf_biquad_coeff_f30_t coeff_rest_gyr;
    vqf_biquad_coeff_f30_t coeff_rest_acc;
    uint32_t init_acc_lp;
    uint32_t init_rest_gyr;
    uint32_t init_rest_acc;
    uint32_t rest_min_samples;
    uint32_t inv_acc_ts; /* 1/Ts integer */
    uint32_t gyr_hz;
    uint32_t acc_hz;
} vqf_fixed_state_t;

static vqf_fixed_state_t st;

FLASH_NZW static void set_identity_p(int32_t p0)
{
    memset(st.biasP_f18, 0, sizeof(st.biasP_f18));
    st.biasP_f18[0] = p0;
    st.biasP_f18[4] = p0;
    st.biasP_f18[8] = p0;
}

FLASH_NZW void vqf_fixed_init(const vqf_fixed_config_t *cfg)
{
    memset(&st, 0, sizeof(st));
    st.gyrQuat[0] = VQF_F30_ONE;
    st.accQuat[0] = VQF_F30_ONE;
    st.gyr_hz = (cfg && cfg->gyr_hz) ? cfg->gyr_hz : 1000u;
    st.acc_hz = (cfg && cfg->acc_hz) ? cfg->acc_hz : 1000u;

    st.coeff_acc_lp = (vqf_biquad_coeff_f30_t){
        VQF_C1K_ACC_LP_B0, VQF_C1K_ACC_LP_B1, VQF_C1K_ACC_LP_B2,
        VQF_C1K_ACC_LP_A1, VQF_C1K_ACC_LP_A2};
    st.init_acc_lp = VQF_C1K_ACC_LP_INIT_SAMPLES;
    st.coeff_rest_acc = (vqf_biquad_coeff_f30_t){
        VQF_C1K_REST_LP_B0, VQF_C1K_REST_LP_B1, VQF_C1K_REST_LP_B2,
        VQF_C1K_REST_LP_A1, VQF_C1K_REST_LP_A2};
    st.init_rest_acc = VQF_C1K_REST_LP_INIT_SAMPLES;
    st.rest_min_samples = VQF_C1K_REST_MIN_SAMPLES;
    st.inv_acc_ts = (st.acc_hz > 0u) ? st.acc_hz : VQF_C1K_INV_ACC_TS;

    if (st.gyr_hz >= 3500u) {
        st.dt_half_f30 = VQF_C4K_DT_HALF_F30;
        st.coeff_rest_gyr = (vqf_biquad_coeff_f30_t){
            VQF_C4K_REST_GYR_B0, VQF_C4K_REST_GYR_B1, VQF_C4K_REST_GYR_B2,
            VQF_C4K_REST_GYR_A1, VQF_C4K_REST_GYR_A2};
        st.init_rest_gyr = VQF_C4K_REST_GYR_INIT_SAMPLES;
    } else {
        st.dt_half_f30 = VQF_C1K_DT_HALF_F30;
        st.coeff_rest_gyr = st.coeff_rest_acc;
        st.init_rest_gyr = VQF_C1K_REST_LP_INIT_SAMPLES;
    }

    set_identity_p(VQF_C1K_BIAS_P0_F18);
    st.lastAccLp_f27[2] = (1 << 27); /* 1 g F27 */
}

FLASH_ZW void vqf_fixed_update_gyr_f25(const int32_t gyr_f25[3])
{
    int i;
    /* Rest gyro LP + deviation (no sqrt) */
    {
        uint64_t dev = 0ull;
        for (i = 0; i < 3; i++) {
            int32_t y = vqf_biquad_step(&st.coeff_rest_gyr, &st.restGyrLp[i],
                                        gyr_f25[i], st.init_rest_gyr);
            st.restGyrLp_f25[i] = y;
            int32_t d = gyr_f25[i] - y;
            dev += (uint64_t)((int64_t)d * (int64_t)d);
        }
        st.restGyrDev_q50 = dev;
        if (dev >= VQF_C1K_REST_TH_GYR2_Q50 ||
            vqf_iabs32(st.restGyrLp_f25[0]) > VQF_C1K_BIAS_CLIP_F25 ||
            vqf_iabs32(st.restGyrLp_f25[1]) > VQF_C1K_BIAS_CLIP_F25 ||
            vqf_iabs32(st.restGyrLp_f25[2]) > VQF_C1K_BIAS_CLIP_F25) {
            st.rest_samples = 0u;
            st.restDetected = 0u;
        }
    }

    int32_t w25[3];
    for (i = 0; i < 3; i++) {
        int32_t b25 = vqf_round_asr(st.bias_f29[i], 4u);
        w25[i] = gyr_f25[i] - b25;
    }

    int32_t v30[3];
    for (i = 0; i < 3; i++) {
        v30[i] = vqf_round_asr((int64_t)w25[i] * (int64_t)st.dt_half_f30, 25u);
    }

    int32_t dq[4];
    vqf_quat_delta_from_v_f30(v30, dq);
    vqf_quat_mul_f30(st.gyrQuat, dq, st.gyrQuat);

    if ((++st.gyr_norm_counter & 31u) == 0u) {
        vqf_quat_normalize_f30_if_needed(st.gyrQuat);
    }
}

FLASH_ZW_CODE __attribute__((noinline)) static void bias_kalman_update(const int32_t acc_earth_unit_f30[3])
{
    int i;
    /* P += V on diagonal (also when no measurement) */
    if (st.biasP_f18[0] < VQF_C1K_BIAS_P0_F18) {
        st.biasP_f18[0] += VQF_C1K_BIAS_V_F18;
    }
    if (st.biasP_f18[4] < VQF_C1K_BIAS_P0_F18) {
        st.biasP_f18[4] += VQF_C1K_BIAS_V_F18;
    }
    if (st.biasP_f18[8] < VQF_C1K_BIAS_P0_F18) {
        st.biasP_f18[8] += VQF_C1K_BIAS_V_F18;
    }

    int32_t R[9];
    int32_t e_f29[3];
    uint64_t w_f8[3];
    int use_meas = 0;

    if (st.restDetected) {
        for (i = 0; i < 3; i++) {
            /* restGyr F25 → F29: <<4 */
            int32_t g29 = st.restGyrLp_f25[i] << 4;
            e_f29[i] = g29 - st.bias_f29[i];
        }
        memset(R, 0, sizeof(R));
        R[0] = R[4] = R[8] = VQF_F30_ONE;
        w_f8[0] = w_f8[1] = w_f8[2] = VQF_C1K_BIAS_W_REST_F8;
        use_meas = 1;
    }
#if VQF_FIXED_ENABLE_MOTION_BIAS
    else {
        int32_t q6[4];
        vqf_quat_mul_f30(st.accQuat, st.gyrQuat, q6);
        vqf_quat_to_rotmat_f30(q6, R);

        int32_t biasLp_in[2];
        biasLp_in[0] = vqf_round_asr(
            (int64_t)R[0] * st.bias_f29[0] + (int64_t)R[1] * st.bias_f29[1] +
                (int64_t)R[2] * st.bias_f29[2],
            30u);
        biasLp_in[1] = vqf_round_asr(
            (int64_t)R[3] * st.bias_f29[0] + (int64_t)R[4] * st.bias_f29[1] +
                (int64_t)R[5] * st.bias_f29[2],
            30u);

        int32_t R_lp[9];
        for (i = 0; i < 9; i++) {
            R_lp[i] = vqf_biquad_step(&st.coeff_acc_lp, &st.motionRlp[i], R[i],
                                      st.init_acc_lp);
        }
        int32_t biasLp[2];
        biasLp[0] = vqf_biquad_step(&st.coeff_acc_lp, &st.motionBiasLp[0],
                                    biasLp_in[0], st.init_acc_lp);
        biasLp[1] = vqf_biquad_step(&st.coeff_acc_lp, &st.motionBiasLp[1],
                                    biasLp_in[1], st.init_acc_lp);
        memcpy(R, R_lp, sizeof(R));

        /* e = motion measurement (design §12); int64 then clip to F29 */
        int64_t e0 = -(int64_t)vqf_round_asr(
            (int64_t)acc_earth_unit_f30[1] * (int64_t)st.inv_acc_ts, 1u);
        e0 += biasLp[0];
        e0 -= vqf_round_asr((int64_t)R[0] * st.bias_f29[0] + (int64_t)R[1] * st.bias_f29[1] +
                                (int64_t)R[2] * st.bias_f29[2],
                            30u);

        int64_t e1 = (int64_t)vqf_round_asr(
            (int64_t)acc_earth_unit_f30[0] * (int64_t)st.inv_acc_ts, 1u);
        e1 += biasLp[1];
        e1 -= vqf_round_asr((int64_t)R[3] * st.bias_f29[0] + (int64_t)R[4] * st.bias_f29[1] +
                                (int64_t)R[5] * st.bias_f29[2],
                            30u);

        int64_t e2 = -vqf_round_asr((int64_t)R[6] * st.bias_f29[0] + (int64_t)R[7] * st.bias_f29[1] +
                                        (int64_t)R[8] * st.bias_f29[2],
                                    30u);

        e_f29[0] = vqf_clip_s32(vqf_sat_s32(e0), -VQF_C1K_BIAS_CLIP_F29, VQF_C1K_BIAS_CLIP_F29);
        e_f29[1] = vqf_clip_s32(vqf_sat_s32(e1), -VQF_C1K_BIAS_CLIP_F29, VQF_C1K_BIAS_CLIP_F29);
        e_f29[2] = vqf_clip_s32(vqf_sat_s32(e2), -VQF_C1K_BIAS_CLIP_F29, VQF_C1K_BIAS_CLIP_F29);
        w_f8[0] = VQF_C1K_BIAS_W_MOTION_F8;
        w_f8[1] = VQF_C1K_BIAS_W_MOTION_F8;
        w_f8[2] = VQF_C1K_BIAS_W_VERT_F8;
        use_meas = 1;
    }
#else
    (void)acc_earth_unit_f30;
#endif

    if (!use_meas) {
        return;
    }

    for (i = 0; i < 3; i++) {
        e_f29[i] = vqf_clip_s32(e_f29[i], -VQF_C1K_BIAS_CLIP_F29, VQF_C1K_BIAS_CLIP_F29);
    }

    /* B = P * R^T  (F18 result after shifts); A = W + R*B in F8 int64 */
    int64_t B[9];
    int64_t A[9];
    /* P is F18, R F30: (P*R)>>30 → F18 */
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            /* B[r,c] = sum_k P[r,k] * R[c,k]  (R^T[k,c]=R[c,k]) */
            int64_t s = 0;
            s += (int64_t)st.biasP_f18[r * 3 + 0] * R[c * 3 + 0];
            s += (int64_t)st.biasP_f18[r * 3 + 1] * R[c * 3 + 1];
            s += (int64_t)st.biasP_f18[r * 3 + 2] * R[c * 3 + 2];
            int32_t b18 = vqf_round_asr(s, 30u);
            B[r * 3 + c] = ((int64_t)b18 << 8) >> 10; /* F18 → F8 approx: >>10 then we'll use as F8 */
            /* cleaner: F18 to F8 is >>10 */
            B[r * 3 + c] = (int64_t)b18 >> 10;
        }
    }
    /* R*B: R F30 * B F8 → shift 30 → F8 */
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            int64_t s = (int64_t)R[r * 3 + 0] * B[0 * 3 + c] +
                        (int64_t)R[r * 3 + 1] * B[1 * 3 + c] +
                        (int64_t)R[r * 3 + 2] * B[2 * 3 + c];
            A[r * 3 + c] = vqf_round_asr(s, 30u);
        }
    }
    A[0] += (int64_t)w_f8[0];
    A[4] += (int64_t)w_f8[1];
    A[8] += (int64_t)w_f8[2];
    /* Force symmetry */
    A[1] = A[3] = (A[1] + A[3]) / 2;
    A[2] = A[6] = (A[2] + A[6]) / 2;
    A[5] = A[7] = (A[5] + A[7]) / 2;

    int32_t K[9];
    if (!vqf_ldlt_solve_3x3_k(A, B, K)) {
        return;
    }

    /* bias += K * e ; K F30, e F29 → product >>30 → F29 */
    for (int r = 0; r < 3; r++) {
        int64_t s = (int64_t)K[r * 3 + 0] * e_f29[0] + (int64_t)K[r * 3 + 1] * e_f29[1] +
                    (int64_t)K[r * 3 + 2] * e_f29[2];
        st.bias_f29[r] = vqf_clip_s32(st.bias_f29[r] + vqf_round_asr(s, 30u),
                                      -VQF_C1K_BIAS_CLIP_F29, VQF_C1K_BIAS_CLIP_F29);
    }

    /* P -= (K*R)*P ; work in F18 */
    int32_t KR[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            int64_t s = (int64_t)K[r * 3 + 0] * R[0 * 3 + c] + (int64_t)K[r * 3 + 1] * R[1 * 3 + c] +
                        (int64_t)K[r * 3 + 2] * R[2 * 3 + c];
            KR[r * 3 + c] = vqf_round_asr(s, 30u); /* F30 */
        }
    }
    int32_t KRP[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            int64_t s = (int64_t)KR[r * 3 + 0] * st.biasP_f18[0 * 3 + c] +
                        (int64_t)KR[r * 3 + 1] * st.biasP_f18[1 * 3 + c] +
                        (int64_t)KR[r * 3 + 2] * st.biasP_f18[2 * 3 + c];
            KRP[r * 3 + c] = vqf_round_asr(s, 30u); /* F18 */
        }
    }
    for (i = 0; i < 9; i++) {
        st.biasP_f18[i] -= KRP[i];
    }
    /* Symmetrize + clamp diag */
    st.biasP_f18[1] = st.biasP_f18[3] = (st.biasP_f18[1] + st.biasP_f18[3]) / 2;
    st.biasP_f18[2] = st.biasP_f18[6] = (st.biasP_f18[2] + st.biasP_f18[6]) / 2;
    st.biasP_f18[5] = st.biasP_f18[7] = (st.biasP_f18[5] + st.biasP_f18[7]) / 2;
    if (st.biasP_f18[0] < 0) {
        st.biasP_f18[0] = 0;
    }
    if (st.biasP_f18[4] < 0) {
        st.biasP_f18[4] = 0;
    }
    if (st.biasP_f18[8] < 0) {
        st.biasP_f18[8] = 0;
    }
}

FLASH_ZW void vqf_fixed_update_acc_f27(const int32_t acc_g_f27[3])
{
    if (acc_g_f27[0] == 0 && acc_g_f27[1] == 0 && acc_g_f27[2] == 0) {
        return;
    }

    int i;
    uint64_t adev = 0ull;
    for (i = 0; i < 3; i++) {
        int32_t y = vqf_biquad_step(&st.coeff_rest_acc, &st.restAccLp[i], acc_g_f27[i],
                                    st.init_rest_acc);
        st.restAccLp_f27[i] = y;
        int32_t d = acc_g_f27[i] - y;
        adev += (uint64_t)((int64_t)d * (int64_t)d);
    }
    st.restAccDev_q54 = adev;
    if (adev >= VQF_C1K_REST_TH_ACC2_Q54) {
        st.rest_samples = 0u;
        st.restDetected = 0u;
    } else {
        if (st.rest_samples < 0xffffffffu) {
            st.rest_samples++;
        }
        if (st.rest_samples >= st.rest_min_samples) {
            st.restDetected = 1u;
        }
    }

    int32_t acc_earth[3];
    vqf_quat_rotate_vec(st.gyrQuat, acc_g_f27, acc_earth);
    for (i = 0; i < 3; i++) {
        st.lastAccLp_f27[i] = vqf_biquad_step(&st.coeff_acc_lp, &st.accLp[i], acc_earth[i],
                                              st.init_acc_lp);
    }

    int32_t a6[3];
    vqf_quat_rotate_vec(st.accQuat, st.lastAccLp_f27, a6);
    int32_t u30[3];
    if (!vqf_normalize_f27_to_f30(a6, u30)) {
        return;
    }

    /* Inclination correction */
    int32_t one_plus = u30[2] + VQF_F30_ONE;
    if (one_plus < 0) {
        one_plus = 0;
    }
    int32_t qw = vqf_isqrt_f30(one_plus >> 1);
    int32_t corr[4];
    const int32_t eps = VQF_F30_ONE / 100000;
    if (qw > eps) {
        corr[0] = qw;
        corr[1] = (int32_t)(((int64_t)(u30[1] >> 1) << 30) / qw);
        corr[2] = (int32_t)(((int64_t)(-(u30[0] >> 1)) << 30) / qw);
        corr[3] = 0;
    } else {
        corr[0] = 0;
        corr[1] = VQF_F30_ONE;
        corr[2] = 0;
        corr[3] = 0;
    }
    vqf_quat_mul_f30(corr, st.accQuat, st.accQuat);
    if ((++st.acc_norm_counter & 15u) == 0u) {
        vqf_quat_normalize_f30(st.accQuat);
    } else {
        vqf_quat_normalize_f30_if_needed(st.accQuat);
    }

    bias_kalman_update(u30);
}

FLASH_ZW_CODE void vqf_fixed_get_quat6d_f30(int32_t q_out[4])
{
    vqf_quat_mul_f30(st.accQuat, st.gyrQuat, q_out);
    vqf_quat_normalize_f30(q_out);
}

FLASH_ZW_CODE void vqf_fixed_get_bias_f29(int32_t bias_out[3])
{
    bias_out[0] = st.bias_f29[0];
    bias_out[1] = st.bias_f29[1];
    bias_out[2] = st.bias_f29[2];
}

FLASH_ZW_CODE bool vqf_fixed_get_rest_detected(void)
{
    return st.restDetected != 0u;
}

/* ---- Euler millideg (same approach as former vqfx) ---- */

FLASH_ZW_CODE static int32_t atan_unit_mdeg(int32_t y_q16, int32_t x_q16)
{
    if (x_q16 == 0 && y_q16 == 0) {
        return 0;
    }
    if (x_q16 == 0) {
        return 90000;
    }
    if (y_q16 == 0) {
        return 0;
    }
    int32_t a_q16;
    int swapped = 0;
    if (y_q16 > x_q16) {
        a_q16 = (int32_t)(((int64_t)x_q16 << 16) / y_q16);
        swapped = 1;
    } else {
        a_q16 = (int32_t)(((int64_t)y_q16 << 16) / x_q16);
    }
    int32_t a2 = (int32_t)(((int64_t)a_q16 * a_q16) >> 16);
    int32_t a3 = (int32_t)(((int64_t)a2 * a_q16) >> 16);
    int32_t a5 = (int32_t)(((int64_t)a3 * a2) >> 16);
    int32_t rad_q16 = a_q16 - a3 / 3 + a5 / 5;
    int32_t mdeg = (int32_t)(((int64_t)rad_q16 * 57296) >> 16);
    if (swapped) {
        mdeg = 90000 - mdeg;
    }
    return vqf_clip_s32(mdeg, 0, 90000);
}

FLASH_ZW_CODE static int32_t atan2_mdeg(int32_t y, int32_t x)
{
    int32_t ay = vqf_iabs32(y);
    int32_t ax = vqf_iabs32(x);
    int32_t a = atan_unit_mdeg(ay, ax);
    if (x >= 0 && y >= 0) {
        return a;
    }
    if (x < 0 && y >= 0) {
        return 180000 - a;
    }
    if (x < 0 && y < 0) {
        return -180000 + a;
    }
    return -a;
}

FLASH_ZW_CODE static int32_t asin_mdeg(int32_t s_q16)
{
    s_q16 = vqf_clip_s32(s_q16, -(1 << 16), (1 << 16));
    int32_t s2 = (int32_t)(((int64_t)s_q16 * s_q16) >> 16);
    int32_t one_m = (1 << 16) - s2;
    if (one_m < 0) {
        one_m = 0;
    }
    uint32_t r = vqf_isqrt_u32((uint32_t)one_m);
    int32_t c_q16 = (int32_t)(r << 8);
    if (c_q16 < 1) {
        return (s_q16 >= 0) ? 90000 : -90000;
    }
    return atan2_mdeg(s_q16, c_q16);
}

FLASH_ZW_CODE void vqf_fixed_get_euler_mdeg(int32_t *roll_mdeg, int32_t *pitch_mdeg, int32_t *yaw_mdeg)
{
    int32_t q[4];
    vqf_fixed_get_quat6d_f30(q);
    int32_t w = q[0], x = q[1], y = q[2], z = q[3];
    int32_t sinr_cosp = (vqf_mul_f30(w, x) + vqf_mul_f30(y, z)) << 1;
    int32_t cosr_cosp = VQF_F30_ONE - ((vqf_mul_f30(x, x) + vqf_mul_f30(y, y)) << 1);
    *roll_mdeg = atan2_mdeg(sinr_cosp >> 14, cosr_cosp >> 14);
    int32_t sinp = (vqf_mul_f30(w, y) - vqf_mul_f30(z, x)) << 1;
    *pitch_mdeg = asin_mdeg(sinp >> 14);
    int32_t siny_cosp = (vqf_mul_f30(w, z) + vqf_mul_f30(x, y)) << 1;
    int32_t cosy_cosp = VQF_F30_ONE - ((vqf_mul_f30(y, y) + vqf_mul_f30(z, z)) << 1);
    *yaw_mdeg = atan2_mdeg(siny_cosp >> 14, cosy_cosp >> 14);
}

FLASH_NZW void vqf_fixed_quat_to_float(const int32_t q_f30[4], float q_out[4])
{
    const float s = 1.0f / (float)VQF_F30_ONE;
    q_out[0] = (float)q_f30[0] * s;
    q_out[1] = (float)q_f30[1] * s;
    q_out[2] = (float)q_f30[2] * s;
    q_out[3] = (float)q_f30[3] * s;
}
