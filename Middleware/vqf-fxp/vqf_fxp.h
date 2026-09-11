/**
 * Fixed-point 6DOF attitude filter — VQF-structured (not a full Laidig VQF port).
 *
 * Structure mirrors dusking1/vqf-c 6DOF hot path:
 *   - strapdown gyro quaternion integration (gyrQuat)
 *   - inertial-frame accel LP + inclination correction (accQuat)
 *   - light gyro bias estimator
 * Quaternion = accQuat ⊗ gyrQuat (getQuat6D equivalent).
 *
 * Q-format (see README):
 *   quat  : Q30  (1.0 == 1<<30)
 *   rates : Q16 rad/s
 *   accel : Q16 m/s²
 *
 * Hot path uses RISC-V M 32×32→64 mul (mul/mulh) and hardware div; no soft-float / libm.
 */
#ifndef VQF_FXP_H
#define VQF_FXP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 1.0 in Q30. */
#define VQFX_ONE_Q30          (1 << 30)
/** 1.0 in Q16. */
#define VQFX_ONE_Q16          (1 << 16)

typedef int32_t vqfx_q16_t;
typedef int32_t vqfx_q30_t;

/**
 * Initialise filter for fixed sample rate (Hz). Cold / NZW.
 * @param sample_hz typically 1000
 */
void vqfx_init(uint32_t sample_hz);

/**
 * Gyro prediction step (rad/s Q16). FLASH_ZW.
 * Removes bias, integrates δq into gyrQuat, normalises.
 */
void vqfx_update_gyr(const vqfx_q16_t gyr_rads_q16[3]);

/**
 * Accel inclination correction (m/s² Q16). FLASH_ZW.
 * Ignores [0,0,0]. Updates accQuat and optional bias.
 */
void vqfx_update_acc(const vqfx_q16_t acc_mps2_q16[3]);

/**
 * 6DOF quaternion wxyz in Q30. FLASH_ZW.
 */
void vqfx_get_quat6d(vqfx_q30_t q_out[4]);

/**
 * Aerospace ZYX Euler from current 6D quat → millidegrees (int32, typically ±180000).
 * FLASH_ZW. Uses fixed-point atan2/asin approximations (no libm).
 */
void vqfx_get_euler_mdeg(int32_t *roll_mdeg, int32_t *pitch_mdeg, int32_t *yaw_mdeg);

/**
 * Convert Q30 quaternion to float (NZW / debug only — not on 1 kHz hot path).
 */
void vqfx_quat_to_float(const vqfx_q30_t q_q30[4], float q_out[4]);

#ifdef __cplusplus
}
#endif

#endif /* VQF_FXP_H */
