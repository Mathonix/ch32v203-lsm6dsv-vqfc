/**
 * Deprecated thin wrappers around vqf_fixed (Q16 legacy API).
 * Prefer vqf_fixed_* + lsm6dsv_read_acc_gyr_fixed on the hot path.
 */
#include "vqf_fxp.h"
#include "vqf_fixed.h"
#include "flash_zw.h"
#include "flash_nzw.h"

FLASH_NZW void vqfx_init(uint32_t sample_hz)
{
    vqf_fixed_config_t cfg = {
        .gyr_hz = sample_hz ? sample_hz : 1000u,
        .acc_hz = sample_hz ? sample_hz : 1000u,
        .mag_hz = 100u,
    };
    vqf_fixed_init(&cfg);
}

FLASH_ZW_CODE void vqfx_update_gyr(const vqfx_q16_t gyr_rads_q16[3])
{
    int32_t g25[3] = { gyr_rads_q16[0] << 9, gyr_rads_q16[1] << 9, gyr_rads_q16[2] << 9 };
    vqf_fixed_update_gyr_f25(g25);
}

FLASH_ZW_CODE void vqfx_update_acc(const vqfx_q16_t acc_mps2_q16[3])
{
    /* m/s² Q16 → g F27: * 2^27 / (9.80665 * 2^16) ≈ * 209 */
    int32_t a27[3];
    for (int i = 0; i < 3; i++) {
        a27[i] = (int32_t)(((int64_t)acc_mps2_q16[i] * 209) >> 0);
    }
    vqf_fixed_update_acc_f27(a27);
}

FLASH_ZW_CODE void vqfx_get_quat6d(vqfx_q30_t q_out[4])
{
    vqf_fixed_get_quat6d_f30(q_out);
}

FLASH_ZW_CODE void vqfx_get_euler_mdeg(int32_t *roll_mdeg, int32_t *pitch_mdeg, int32_t *yaw_mdeg)
{
    vqf_fixed_get_euler_mdeg(roll_mdeg, pitch_mdeg, yaw_mdeg);
}

FLASH_NZW void vqfx_quat_to_float(const vqfx_q30_t q_q30[4], float q_out[4])
{
    vqf_fixed_quat_to_float(q_q30, q_out);
}
