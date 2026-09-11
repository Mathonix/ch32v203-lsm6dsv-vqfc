/**
 * Fixed-point VQF-structured 6DOF filter for QingKe V4B (rv32imac).
 *
 * Honesty label: VQF-structured fxp — same state split (gyrQuat / accQuat),
 * gyro integrate + accel tilt correction + simple bias — NOT a bit-exact
 * fixed-point port of full Laidig VQF (no Kalman biasP / rest-LP / mag).
 *
 * All hot-path arithmetic is integer; GCC emits mul/mulh for int64 products.
 */
#include "vqf_fxp.h"
#include "flash_zw.h"
#include "flash_nzw.h"

#include <string.h>

/* ---- Q helpers (RV M: 32×32→64 via mul/mulh) ---- */

static inline int32_t q16_mul(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * (int64_t)b) >> 16);
}

static inline int32_t q30_mul(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * (int64_t)b) >> 30);
}

static inline int32_t q30_mul_q16(int32_t a_q30, int32_t b_q16)
{
    return (int32_t)(((int64_t)a_q30 * (int64_t)b_q16) >> 16);
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static inline int32_t iabs32(int32_t x)
{
    return (x < 0) ? -x : x;
}

/** Integer sqrt for uint32 (Newton). Uses hardware div. */
FLASH_ZW static uint32_t isqrt_u32(uint32_t x)
{
    if (x == 0u) {
        return 0u;
    }
    uint32_t r = x;
    /* Initial guess: 2^ceil(log2(x)/2) */
    uint32_t s = 1u << ((32u - (uint32_t)__builtin_clz(x) + 1u) >> 1);
    for (int i = 0; i < 5; i++) {
        s = (s + x / s) >> 1;
    }
    /* One more if overshot */
    if (s * s > x && s > 0u) {
        s--;
    }
    (void)r;
    return s;
}

/** Reciprocal sqrt for Q30 vector norm² → scale so ||v||→1 in Q30. */
FLASH_ZW static int32_t q30_rsqrt_norm2(int64_t norm2_q60)
{
    /* norm2 is sum of Q30*Q30 → Q60. Want 2^30 / sqrt(norm2_as_float).
     * sqrt(norm2_q60) in Q30 domain: isqrt(norm2_q60 >> 30) roughly Q30,
     * better: isqrt(norm2_q60) is Q30, then ONE/that. */
    if (norm2_q60 <= 0) {
        return VQFX_ONE_Q30;
    }
    /* Bring toward 32-bit for isqrt */
    uint64_t n = (uint64_t)norm2_q60;
    uint32_t shift = 0u;
    while (n > 0xffffffffull) {
        n >>= 2;
        shift += 1u;
    }
    uint32_t root = isqrt_u32((uint32_t)n); /* ≈ sqrt(norm2) >> shift */
    if (root == 0u) {
        return VQFX_ONE_Q30;
    }
    /* true_root_q30 ≈ root << shift; scale = 2^30 / true_root */
    /* (1<<30) / (root << shift) = (1<<(30-shift)) / root */
    if (shift >= 30u) {
        return 1;
    }
    return (int32_t)(((int64_t)1 << (30 - (int)shift)) / (int32_t)root);
}

FLASH_ZW static void quat_normalize_q30(int32_t q[4])
{
    int64_t n2 = (int64_t)q[0] * q[0] + (int64_t)q[1] * q[1] +
                 (int64_t)q[2] * q[2] + (int64_t)q[3] * q[3];
    int32_t s = q30_rsqrt_norm2(n2);
    q[0] = q30_mul(q[0], s);
    q[1] = q30_mul(q[1], s);
    q[2] = q30_mul(q[2], s);
    q[3] = q30_mul(q[3], s);
}

FLASH_ZW static void quat_multiply_q30(const int32_t a[4], const int32_t b[4], int32_t out[4])
{
    /* (w1w2 - x1x2 - y1y2 - z1z2, ...) */
    int32_t w = q30_mul(a[0], b[0]) - q30_mul(a[1], b[1]) - q30_mul(a[2], b[2]) - q30_mul(a[3], b[3]);
    int32_t x = q30_mul(a[0], b[1]) + q30_mul(a[1], b[0]) + q30_mul(a[2], b[3]) - q30_mul(a[3], b[2]);
    int32_t y = q30_mul(a[0], b[2]) - q30_mul(a[1], b[3]) + q30_mul(a[2], b[0]) + q30_mul(a[3], b[1]);
    int32_t z = q30_mul(a[0], b[3]) + q30_mul(a[1], b[2]) - q30_mul(a[2], b[1]) + q30_mul(a[3], b[0]);
    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

FLASH_ZW static void quat_rotate_q30_q16(const int32_t q[4], const int32_t v_q16[3], int32_t out_q16[3])
{
    /* Convert v to Q30 (<<14), rotate, >>14 back to Q16 */
    int32_t vq[3] = { v_q16[0] << 14, v_q16[1] << 14, v_q16[2] << 14 };
    /* t = 2 * cross(q.xyz, v) */
    int32_t tx = q30_mul(q[2], vq[2]) - q30_mul(q[3], vq[1]);
    int32_t ty = q30_mul(q[3], vq[0]) - q30_mul(q[1], vq[2]);
    int32_t tz = q30_mul(q[1], vq[1]) - q30_mul(q[2], vq[0]);
    tx <<= 1;
    ty <<= 1;
    tz <<= 1;
    /* v' = v + q.w * t + cross(q.xyz, t) */
    int32_t cx = q30_mul(q[2], tz) - q30_mul(q[3], ty);
    int32_t cy = q30_mul(q[3], tx) - q30_mul(q[1], tz);
    int32_t cz = q30_mul(q[1], ty) - q30_mul(q[2], tx);
    int32_t ox = vq[0] + q30_mul(q[0], tx) + cx;
    int32_t oy = vq[1] + q30_mul(q[0], ty) + cy;
    int32_t oz = vq[2] + q30_mul(q[0], tz) + cz;
    out_q16[0] = ox >> 14;
    out_q16[1] = oy >> 14;
    out_q16[2] = oz >> 14;
}

/**
 * cos/sin of half-angle for |θ| small (1 kHz, FS 2000 dps → |θ|<~0.035 rad).
 * Input angle in Q30 radians; outputs c,s in Q30.
 * Taylor: cos x ≈ 1 - x²/2 + x⁴/24; sin x ≈ x - x³/6
 */
FLASH_ZW static void sincos_half_q30(int32_t angle_q30, int32_t *c_q30, int32_t *s_q30)
{
    int32_t x = angle_q30 >> 1; /* θ/2 */
    int32_t x2 = q30_mul(x, x);
    int32_t x3 = q30_mul(x2, x);
    int32_t x4 = q30_mul(x2, x2);
    /* 1/2 = 1<<29, 1/6 ≈ (1<<30)/6, 1/24 ≈ (1<<30)/24 */
    int32_t cos_term = VQFX_ONE_Q30 - (x2 >> 1) + (int32_t)((int64_t)x4 / 24);
    int32_t sin_term = x - (int32_t)((int64_t)x3 / 6);
    *c_q30 = cos_term;
    *s_q30 = sin_term;
}

/* ---- State ---- */

static int32_t gyr_quat[4];
static int32_t acc_quat[4];
static int32_t bias_q16[3];
static int32_t acc_lp_q16[3];
static int32_t kp_q16;          /* tilt proportional gain (Q16) */
static int32_t beta_q16;        /* bias learning rate (Q16) */
static int32_t lp_alpha_q16;    /* 1st-order LP alpha for tau≈0.5s @1kHz */
static uint32_t sample_hz;
static int32_t bias_clip_q16;   /* ±2 °/s in Q16 rad/s */

FLASH_NZW void vqfx_init(uint32_t hz)
{
    sample_hz = (hz == 0u) ? 1000u : hz;
    gyr_quat[0] = VQFX_ONE_Q30;
    gyr_quat[1] = gyr_quat[2] = gyr_quat[3] = 0;
    acc_quat[0] = VQFX_ONE_Q30;
    acc_quat[1] = acc_quat[2] = acc_quat[3] = 0;
    bias_q16[0] = bias_q16[1] = bias_q16[2] = 0;
    acc_lp_q16[0] = acc_lp_q16[1] = 0;
    acc_lp_q16[2] = 643017; /* ≈9.80665 m/s² in Q16 */
    /* kp≈1.0 (Mahony-like tilt), beta≈0.01, alpha≈ dt/tau with tau=0.5 → 0.002 */
    kp_q16 = VQFX_ONE_Q16;                              /* 1.0 */
    beta_q16 = VQFX_ONE_Q16 / 100;                      /* 0.01 */
    lp_alpha_q16 = (int32_t)((65536u * 2u) / sample_hz); /* ≈2/hz → tau~0.5 s */
    if (lp_alpha_q16 < 1) {
        lp_alpha_q16 = 1;
    }
    /* 2 deg/s → rad/s Q16: 2 * pi/180 * 65536 ≈ 2289 */
    bias_clip_q16 = 2289;
}

FLASH_ZW void vqfx_update_gyr(const vqfx_q16_t gyr_rads_q16[3])
{
    int32_t gx = gyr_rads_q16[0] - bias_q16[0];
    int32_t gy = gyr_rads_q16[1] - bias_q16[1];
    int32_t gz = gyr_rads_q16[2] - bias_q16[2];

    /* ||ω|| in Q16 */
    int64_t n2 = (int64_t)gx * gx + (int64_t)gy * gy + (int64_t)gz * gz;
    uint32_t n_u = isqrt_u32((uint32_t)((n2 > 0xffffffffll) ? 0xffffffffu : (uint32_t)n2));
    int32_t gyr_norm = (int32_t)n_u; /* Q16 */

    if (gyr_norm < 8) { /* ~0.0001 rad/s threshold */
        return;
    }

    /* angle = ||ω|| * dt → Q30: (gyr_norm_q16 * 2^14) / sample_hz */
    int32_t angle_q30 = (int32_t)(((int64_t)gyr_norm << 14) / (int32_t)sample_hz);

    int32_t c, s;
    sincos_half_q30(angle_q30, &c, &s);

    /* s_axis = sin(θ/2) / ||ω||  → apply to ω components
     * s is Q30, gyr_norm Q16 → (s << 16) / gyr_norm gives Q30 scale for ω_q16
     * δq.xyz = (s/||ω||) * ω = s_q30 * ω_q16 / gyr_norm_q16
     */
    int32_t inv_n = (int32_t)(((int64_t)1 << 30) / gyr_norm); /* Q14 actually? */
    /* Want (s * gx) / gyr_norm in Q30: s_q30 * gx_q16 / gyr_norm_q16
     * = (s * gx) >> 16  then need /gyr_norm... use (s * gx) / gyr_norm >> 16 */
    int32_t sx = (int32_t)(((int64_t)s * gx) / gyr_norm);
    int32_t sy = (int32_t)(((int64_t)s * gy) / gyr_norm);
    int32_t sz = (int32_t)(((int64_t)s * gz) / gyr_norm);
    /* s*gx is Q46, /gyr_norm(Q16) → Q30. Good. */
    (void)inv_n;

    int32_t step[4] = { c, sx, sy, sz };
    int32_t tmp[4];
    quat_multiply_q30(gyr_quat, step, tmp);
    gyr_quat[0] = tmp[0];
    gyr_quat[1] = tmp[1];
    gyr_quat[2] = tmp[2];
    gyr_quat[3] = tmp[3];
    quat_normalize_q30(gyr_quat);
}

FLASH_ZW void vqfx_update_acc(const vqfx_q16_t acc_mps2_q16[3])
{
    if (acc_mps2_q16[0] == 0 && acc_mps2_q16[1] == 0 && acc_mps2_q16[2] == 0) {
        return;
    }

    /* Filter accel in inertial frame (rotate by gyrQuat) — VQF style */
    int32_t acc_earth[3];
    quat_rotate_q30_q16(gyr_quat, acc_mps2_q16, acc_earth);

    /* 1st-order LP: lp += α (x - lp) */
    for (int i = 0; i < 3; i++) {
        int32_t d = acc_earth[i] - acc_lp_q16[i];
        acc_lp_q16[i] += q16_mul(lp_alpha_q16, d);
    }

    /* Transform LP accel into 6D earth frame via accQuat */
    int32_t a6[3];
    quat_rotate_q30_q16(acc_quat, acc_lp_q16, a6);

    /* Normalise a6 to unit (Q16 unit = 1<<16) */
    int64_t n2 = (int64_t)a6[0] * a6[0] + (int64_t)a6[1] * a6[1] + (int64_t)a6[2] * a6[2];
    uint32_t nu = isqrt_u32((uint32_t)((n2 > 0xffffffffll) ? 0xffffffffu : (uint32_t)n2));
    if (nu < 16) {
        return;
    }
    /* a6_unit_q16 = a6 * (1<<16) / nu */
    int32_t ux = (int32_t)(((int64_t)a6[0] << 16) / (int32_t)nu);
    int32_t uy = (int32_t)(((int64_t)a6[1] << 16) / (int32_t)nu);
    int32_t uz = (int32_t)(((int64_t)a6[2] << 16) / (int32_t)nu);

    /* Inclination correction quat (VQF): qw = sqrt((az+1)/2), qx=0.5*ay/qw, qy=-0.5*ax/qw
     * Work in Q30: uz_q30 = uz << 14; */
    int32_t uz_q30 = uz << 14;
    int32_t ux_q30 = ux << 14;
    int32_t uy_q30 = uy << 14;
    int32_t one_plus = uz_q30 + VQFX_ONE_Q30;
    if (one_plus < 0) {
        one_plus = 0;
    }
    /* sqrt((az+1)/2) = sqrt(one_plus/2). isqrt of (one_plus>>1) as Q30 value:
     * one_plus is Q30; (one_plus/2) Q30; sqrt of float = sqrt(val/2^30)*2^15...
     * Use: root = isqrt(one_plus >> 1) then interpret — simpler Newton on Q30. */
    uint32_t half = (uint32_t)(one_plus >> 1);
    /* Map Q30 value to integer domain for isqrt: want sqrt(half_q30) in Q30
     * sqrt(h / 2^30) * 2^30 = 2^15 * sqrt(h) ≈ (isqrt(h) << 15) if h is the Q30 int.
     */
    uint32_t r = isqrt_u32(half);
    int32_t qw = (int32_t)(r << 15); /* ≈ Q30 */
    /* Refine: qw should be ~2^30 when uz=1. When uz=1, one_plus=2^31, half=2^30,
     * isqrt(2^30)=2^15, <<15 = 2^30. Good. */

    int32_t corr[4];
    const int32_t eps = VQFX_ONE_Q30 / 100000; /* ~1e-5 */
    if (qw > eps) {
        corr[0] = qw;
        /* 0.5 * uy / qw : (uy_q30/2) / qw_as_float = (uy_q30 << 29) / qw ? 
         * (uy_q30 / 2) * (2^30 / qw) / 2^30 = (uy_q30 >> 1) * ONE / qw
         * result Q30: ((uy_q30 >> 1) << 30) / qw  — careful overflow:
         * use ((int64_t)(uy_q30 >> 1) << 30) / qw */
        corr[1] = (int32_t)(((int64_t)(uy_q30 >> 1) << 30) / qw);
        corr[2] = (int32_t)(((int64_t)(-(ux_q30 >> 1)) << 30) / qw);
        corr[3] = 0;
    } else {
        /* 180° singularity: identity-like flip */
        corr[0] = 0;
        corr[1] = VQFX_ONE_Q30;
        corr[2] = 0;
        corr[3] = 0;
    }

    int32_t tmp[4];
    quat_multiply_q30(corr, acc_quat, tmp);
    acc_quat[0] = tmp[0];
    acc_quat[1] = tmp[1];
    acc_quat[2] = tmp[2];
    acc_quat[3] = tmp[3];
    quat_normalize_q30(acc_quat);

    /* Simple bias: pull bias toward body-frame tilt error × gain (rest-light)
     * error ~ cross(estimated_gravity, measured) in body — use a6 unit vs [0,0,1]
     * body error rates ≈ kp * (-uy, ux, 0) scaled; integrate into bias slowly.
     */
    int32_t ex = -uy; /* Q16 */
    int32_t ey = ux;
    bias_q16[0] = clamp_i32(bias_q16[0] + q16_mul(beta_q16, ex), -bias_clip_q16, bias_clip_q16);
    bias_q16[1] = clamp_i32(bias_q16[1] + q16_mul(beta_q16, ey), -bias_clip_q16, bias_clip_q16);
    (void)kp_q16;
}

FLASH_ZW void vqfx_get_quat6d(vqfx_q30_t q_out[4])
{
    quat_multiply_q30(acc_quat, gyr_quat, q_out);
    quat_normalize_q30(q_out);
}

/* ---- Fixed-point atan2 / asin → millidegrees ---- */

/**
 * atan(|z|) for z in Q16, z in [0,1] → millideg [0,45000].
 * Rational approx: atan(z)° ≈ z*(45 - z*(z-1)*(14 + 3.83*z)) roughly;
 * Use degree polynomial on Q16:
 *   atan_deg ≈ (a1*z - a3*z^3 + a5*z^5) * (180/pi)
 * Coefficients in Q16 for rad, then * (180000/pi) for mdeg — do in one go.
 *
 * Simpler CORDIC-ish iterative in millideg.
 */
FLASH_ZW static int32_t atan_unit_mdeg(int32_t y_q16, int32_t x_q16)
{
    /* Both >=0, return atan(y/x) in millideg [0,90000] */
    if (x_q16 == 0 && y_q16 == 0) {
        return 0;
    }
    if (x_q16 == 0) {
        return 90000;
    }
    if (y_q16 == 0) {
        return 0;
    }

    /* Bring to |a|<=1 where a = min/max */
    int32_t a_q16;
    int swapped = 0;
    if (y_q16 > x_q16) {
        a_q16 = (int32_t)(((int64_t)x_q16 << 16) / y_q16);
        swapped = 1;
    } else {
        a_q16 = (int32_t)(((int64_t)y_q16 << 16) / x_q16);
    }

    /* atan(a) mdeg ≈ a_q16 * 57300 / 65536   (180000/pi ≈ 57295.8)
     * corrected: atan(a)≈ a - a^3/3 + a^5/5 (rad) → *57296
     */
    int32_t a2 = q16_mul(a_q16, a_q16);
    int32_t a3 = q16_mul(a2, a_q16);
    int32_t a5 = q16_mul(a3, a2);
    /* rad_q16 ≈ a - a3/3 + a5/5 */
    int32_t rad_q16 = a_q16 - a3 / 3 + a5 / 5;
    int32_t mdeg = (int32_t)(((int64_t)rad_q16 * 57296) >> 16);
    if (swapped) {
        mdeg = 90000 - mdeg;
    }
    return clamp_i32(mdeg, 0, 90000);
}

FLASH_ZW static int32_t atan2_mdeg(int32_t y, int32_t x)
{
    /* y,x any Q16 (or same scale) */
    int32_t ay = iabs32(y);
    int32_t ax = iabs32(x);
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
    /* x>=0 && y<0 */
    return -a;
}

FLASH_ZW static int32_t asin_mdeg(int32_t s_q16)
{
    /* s in Q16, |s|<=1 → asin in millideg */
    s_q16 = clamp_i32(s_q16, -VQFX_ONE_Q16, VQFX_ONE_Q16);
    /* asin(s) = atan(s / sqrt(1-s²)) */
    int32_t s2 = q16_mul(s_q16, s_q16);
    int32_t one_m = VQFX_ONE_Q16 - s2;
    if (one_m < 0) {
        one_m = 0;
    }
    uint32_t r = isqrt_u32((uint32_t)one_m); /* sqrt of Q16 value → Q8... */
    /* one_m is Q16 (0..65536). isqrt(65536)=256. Want Q16: << 8 */
    int32_t c_q16 = (int32_t)(r << 8);
    if (c_q16 < 1) {
        return (s_q16 >= 0) ? 90000 : -90000;
    }
    int32_t a = atan2_mdeg(s_q16, c_q16);
    return a;
}

FLASH_ZW void vqfx_get_euler_mdeg(int32_t *roll_mdeg, int32_t *pitch_mdeg, int32_t *yaw_mdeg)
{
    int32_t q[4];
    vqfx_get_quat6d(q);
    /* Products in Q30; convert trig args to Q16 (>>14) */
    int32_t w = q[0], x = q[1], y = q[2], z = q[3];

    int32_t sinr_cosp = (q30_mul(w, x) + q30_mul(y, z)) << 1; /* Q30 */
    int32_t cosr_cosp = VQFX_ONE_Q30 - ((q30_mul(x, x) + q30_mul(y, y)) << 1);
    *roll_mdeg = atan2_mdeg(sinr_cosp >> 14, cosr_cosp >> 14);

    int32_t sinp = (q30_mul(w, y) - q30_mul(z, x)) << 1; /* Q30 */
    *pitch_mdeg = asin_mdeg(sinp >> 14);

    int32_t siny_cosp = (q30_mul(w, z) + q30_mul(x, y)) << 1;
    int32_t cosy_cosp = VQFX_ONE_Q30 - ((q30_mul(y, y) + q30_mul(z, z)) << 1);
    *yaw_mdeg = atan2_mdeg(siny_cosp >> 14, cosy_cosp >> 14);
}

FLASH_NZW void vqfx_quat_to_float(const vqfx_q30_t q_q30[4], float q_out[4])
{
    const float s = 1.0f / (float)VQFX_ONE_Q30;
    q_out[0] = (float)q_q30[0] * s;
    q_out[1] = (float)q_q30[1] * s;
    q_out[2] = (float)q_q30[2] * s;
    q_out[3] = (float)q_q30[3] * s;
}
