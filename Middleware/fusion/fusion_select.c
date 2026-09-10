/**
 * Boot-time algorithm selection from Flash flag.
 */
#include "fusion.h"
#include "algo_cfg.h"
#include "flash_nzw.h"

const fusion_algo_t *fusion_active = &fusion_algo_vqf;
algo_id_t fusion_active_id = ALGO_VQF;

FLASH_NZW const char *fusion_algo_name(algo_id_t id)
{
    switch (id) {
    case ALGO_VQF:
        return "VQF (ZW Flash)";
    case ALGO_MAHONY:
        return "Mahony 6DOF (SRAM)";
    case ALGO_COMPLEMENTARY:
        return "Complementary 6DOF (SRAM)";
    default:
        return "unknown";
    }
}

FLASH_NZW void fusion_boot_select(void)
{
    algo_cfg_t cfg;
    fusion_active_id = ALGO_VQF;
    fusion_active = &fusion_algo_vqf;

    if (algo_cfg_read(&cfg) != 0) {
        /* Invalid / erased → default VQF */
        return;
    }

    switch ((algo_id_t)cfg.algo_id) {
    case ALGO_MAHONY:
        fusion_active_id = ALGO_MAHONY;
        fusion_active = &fusion_algo_mahony;
        break;
    case ALGO_COMPLEMENTARY:
        fusion_active_id = ALGO_COMPLEMENTARY;
        fusion_active = &fusion_algo_comp;
        break;
    case ALGO_VQF:
    default:
        fusion_active_id = ALGO_VQF;
        fusion_active = &fusion_algo_vqf;
        break;
    }
}
