/**
 * Unified 6DOF attitude-fusion API.
 *
 * VQF-fxp hot path runs from HOT_RAM (default, boot-copied). Mahony /
 * complementary live in FLASH_NZW with VMA in ALGO_RAM; startup copies
 * them into SRAM and the selected vtable's function pointers target RAM.
 *
 * Units: gyroscope rad/s, accelerometer m/s², quaternion wxyz.
 */
#ifndef FUSION_H
#define FUSION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ALGO_VQF            = 0,
    ALGO_MAHONY         = 1,
    ALGO_COMPLEMENTARY  = 2,
} algo_id_t;

typedef struct {
    void (*init)(float sample_hz);
    void (*update)(const float gyr[3], const float acc[3], float dt); /* rad/s, m/s² */
    void (*get_quat)(float q[4]); /* wxyz */
} fusion_algo_t;

/** Active algorithm (set by fusion_boot_select). */
extern const fusion_algo_t *fusion_active;
extern algo_id_t fusion_active_id;

/** Name string for boot log (static storage). */
const char *fusion_algo_name(algo_id_t id);

/**
 * Read Flash flag, validate, select vtable.
 * Ensures ALGO_RAM has been populated (startup always copies .algo_ram).
 * Invalid/erased flag → ALGO_VQF.
 */
void fusion_boot_select(void);

/** VQF / Mahony / complementary vtables (defined in respective .c files). */
extern const fusion_algo_t fusion_algo_vqf;
extern const fusion_algo_t fusion_algo_mahony;
extern const fusion_algo_t fusion_algo_comp;

#ifdef __cplusplus
}
#endif

#endif /* FUSION_H */
