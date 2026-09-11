/**
 * VQF backend: ALGO_VQF → Full VQF fixed-point (vqf_fixed).
 * Float vtable methods are NZW boundary only; 1 kHz path calls vqf_fixed_* directly.
 */
#include "fusion.h"
#include "vqf_fixed.h"
#include "flash_zw.h"
#include "flash_nzw.h"

FLASH_NZW void vqf_fusion_init(float sample_hz)
{
    uint32_t hz = 1000u;
    if (sample_hz >= 1.0f && sample_hz < 100000.0f) {
        hz = (uint32_t)(sample_hz + 0.5f);
    }
    vqf_fixed_config_t cfg = { .gyr_hz = hz, .acc_hz = hz, .mag_hz = 100u };
    vqf_fixed_init(&cfg);
}

FLASH_NZW void vqf_fusion_update(const float gyr[3], const float acc[3], float dt)
{
    (void)dt;
    int32_t g25[3], a27[3];
    for (int i = 0; i < 3; i++) {
        /* gyr rad/s → F25; acc m/s² → g F27 (≈ /9.80665) */
        g25[i] = (int32_t)(gyr[i] * (float)(1 << 25));
        a27[i] = (int32_t)((acc[i] / 9.80665f) * (float)(1 << 27));
    }
    vqf_fixed_update_gyr_f25(g25);
    vqf_fixed_update_acc_f27(a27);
}

FLASH_NZW void vqf_fusion_get_quat(float q[4])
{
    int32_t qq[4];
    vqf_fixed_get_quat6d_f30(qq);
    vqf_fixed_quat_to_float(qq, q);
}

const fusion_algo_t fusion_algo_vqf = {
    .init = vqf_fusion_init,
    .update = vqf_fusion_update,
    .get_quat = vqf_fusion_get_quat,
};
