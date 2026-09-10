/**
 * VQF backend for fusion_algo_t — hot update/get_quat stay in zero-wait Flash.
 */
#include "fusion.h"
#include "vqf.h"
#include "flash_zw.h"
#include "flash_nzw.h"

FLASH_NZW void vqf_fusion_init(float sample_hz)
{
    const float ts = 1.0f / sample_hz;
    /* Large magTs + never call updateMag → 6DOF */
    initVqf(ts, ts, 5.0f);
}

FLASH_ZW void vqf_fusion_update(const float gyr[3], const float acc[3], float dt)
{
    (void)dt; /* VQF uses fixed gyrTs/accTs from init */
    updateGyr(gyr);
    updateAcc(acc);
}

FLASH_ZW void vqf_fusion_get_quat(float q[4])
{
    getQuat6D(q);
}

const fusion_algo_t fusion_algo_vqf = {
    .init = vqf_fusion_init,
    .update = vqf_fusion_update,
    .get_quat = vqf_fusion_get_quat,
};
