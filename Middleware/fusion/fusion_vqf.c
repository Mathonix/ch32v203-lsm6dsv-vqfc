/**
 * VQF backend: ALGO_VQF → fixed-point VQF-structured filter (vqf-fxp).
 *
 * Float vtable methods are NZW (boundary only). The 1 kHz hot path in main.c
 * calls vqfx_* + lsm6dsv_read_acc_gyr_fxp directly — no soft-float.
 */
#include "fusion.h"
#include "vqf_fxp.h"
#include "flash_zw.h"
#include "flash_nzw.h"

FLASH_NZW void vqf_fusion_init(float sample_hz)
{
    uint32_t hz = 1000u;
    if (sample_hz >= 1.0f && sample_hz < 100000.0f) {
        hz = (uint32_t)(sample_hz + 0.5f);
    }
    vqfx_init(hz);
}

/* Float API kept for vtable completeness; not used on default 1 kHz path. */
FLASH_NZW void vqf_fusion_update(const float gyr[3], const float acc[3], float dt)
{
    (void)dt;
    int32_t gq[3], aq[3];
    for (int i = 0; i < 3; i++) {
        /* Q16 from float without libm: scale via integer cast of (x*65536) */
        gq[i] = (int32_t)(gyr[i] * 65536.0f);
        aq[i] = (int32_t)(acc[i] * 65536.0f);
    }
    vqfx_update_gyr(gq);
    vqfx_update_acc(aq);
}

FLASH_NZW void vqf_fusion_get_quat(float q[4])
{
    int32_t qq[4];
    vqfx_get_quat6d(qq);
    vqfx_quat_to_float(qq, q);
}

const fusion_algo_t fusion_algo_vqf = {
    .init = vqf_fusion_init,
    .update = vqf_fusion_update,
    .get_quat = vqf_fusion_get_quat,
};
