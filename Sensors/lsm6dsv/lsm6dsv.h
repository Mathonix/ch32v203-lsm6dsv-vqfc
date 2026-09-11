/**
 * Minimal LSM6DSV driver (SPI) for bare-metal firmware.
 * WHO_AM_I verified against ST PID / datasheet: 0x70.
 * Bus: SPI mode 3, soft CS, read addr |= 0x80.
 */
#ifndef LSM6DSV_H
#define LSM6DSV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** ST LSM6DSV device ID (WHO_AM_I @ 0x0F). */
#define LSM6DSV_WHO_AM_I_VALUE   0x70u

/**
 * Configured XL+GY output data rate (Hz).
 * High-accuracy ODR (HAODR_SEL=1, ODR code 0x9) → true 1000 Hz
 * (standard mode without HAODR would be 960 Hz at the same ODR code).
 */
#define LSM6DSV_ODR_HZ           1000.0f

typedef struct {
    uint8_t unused; /* SPI has no 7-bit address; kept for API stability */
} lsm6dsv_t;

/**
 * Soft-reset, configure ODR 1000 Hz (HAODR), FS ±4 g / ±2000 dps, BDU+IF_INC.
 * @return 0 on success, negative on SPI/config error, -100 on WHO_AM_I mismatch
 */
int lsm6dsv_init(lsm6dsv_t *dev);

/** Read WHO_AM_I register. */
int lsm6dsv_whoami(lsm6dsv_t *dev, uint8_t *id);

/**
 * Read accelerometer in m/s^2 (x,y,z) and gyroscope in rad/s (x,y,z).
 * Uses FS sensitivities: 0.122 mg/LSB (±4g), 70 mdps/LSB (±2000 dps).
 * FLASH_ZW / .text_zw hot path.
 */
int lsm6dsv_read_acc_gyr(lsm6dsv_t *dev, float acc_mps2[3], float gyr_rads[3]);

/**
 * Read XL+GY raw and convert directly to Q16 (no float).
 * Acc: m/s² Q16 (≈78 Q16 / LSB @ ±4 g, 0.122 mg/LSB).
 * Gyr: rad/s Q16 (≈80 Q16 / LSB @ ±2000 dps, 70 mdps/LSB).
 * FLASH_ZW hot path for fixed-point fusion.
 */
int lsm6dsv_read_acc_gyr_fxp(lsm6dsv_t *dev, int32_t acc_q16[3], int32_t gyr_q16[3]);

/**
 * Raw → Full VQF fixed domains (no float):
 *   acc: g F27 (0.122 mg/LSB → ×16375)
 *   gyr: rad/s F25 (70 mdps/LSB → ×40993)
 */
int lsm6dsv_read_acc_gyr_fixed(lsm6dsv_t *dev, int32_t acc_g_f27[3], int32_t gyr_f25[3]);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSV_H */
