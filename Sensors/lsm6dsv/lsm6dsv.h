/**
 * Minimal LSM6DSV driver (SPI) for bare-metal firmware.
 * WHO_AM_I verified against ST PID / datasheet: 0x70.
 * Bus: SPI mode 3, soft CS, read addr |= 0x80.
 *
 * Default ODR (HAODR mode 1): gyroscope 4 kHz, accelerometer 1 kHz
 * (independent ODRs per DS13476 / ArduPilot LSM6DSV HAODR table).
 */
#ifndef LSM6DSV_H
#define LSM6DSV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** ST LSM6DSV device ID (WHO_AM_I @ 0x0F). */
#define LSM6DSV_WHO_AM_I_VALUE   0x70u

/** Gyro HAODR rate (Hz). */
#define LSM6DSV_GYR_ODR_HZ       4000u
/** Accel HAODR rate (Hz). */
#define LSM6DSV_ACC_ODR_HZ       1000u

/**
 * Legacy name: attitude / UART / Mahony sample rate (= accel ODR).
 * Gyro runs faster; VQF fixed uses LSM6DSV_GYR_ODR_HZ independently.
 */
#define LSM6DSV_ODR_HZ           1000.0f

/** STATUS_REG (0x1E) bits */
#define LSM6DSV_STATUS_XLDA      (1u << 0)
#define LSM6DSV_STATUS_GDA       (1u << 1)

typedef struct {
    uint8_t unused; /* SPI has no 7-bit address; kept for API stability */
} lsm6dsv_t;

/**
 * Soft-reset, configure HAODR GY=4 kHz / XL=1 kHz, FS ±4 g / ±2000 dps, BDU+IF_INC.
 * @return 0 on success, negative on SPI/config error, -100 on WHO_AM_I mismatch
 */
int lsm6dsv_init(lsm6dsv_t *dev);

/** Read WHO_AM_I register. */
int lsm6dsv_whoami(lsm6dsv_t *dev, uint8_t *id);

/** Read STATUS_REG (0x1E). FLASH_ZW. */
int lsm6dsv_read_status(lsm6dsv_t *dev, uint8_t *status);

/**
 * Read accelerometer in m/s^2 (x,y,z) and gyroscope in rad/s (x,y,z).
 * Uses FS sensitivities: 0.122 mg/LSB (±4g), 70 mdps/LSB (±2000 dps).
 */
int lsm6dsv_read_acc_gyr(lsm6dsv_t *dev, float acc_mps2[3], float gyr_rads[3]);

/**
 * Read XL+GY raw and convert directly to Q16 (no float). Legacy path.
 */
int lsm6dsv_read_acc_gyr_fxp(lsm6dsv_t *dev, int32_t acc_q16[3], int32_t gyr_q16[3]);

/**
 * Raw → Full VQF fixed domains (no float):
 *   acc: g F27 (0.122 mg/LSB → ×16375)
 *   gyr: rad/s F25 (70 mdps/LSB → ×40993)
 */
int lsm6dsv_read_acc_gyr_fixed(lsm6dsv_t *dev, int32_t acc_g_f27[3], int32_t gyr_f25[3]);

/** Gyro-only raw → F25 (clears GDA). FLASH_ZW. */
int lsm6dsv_read_gyr_fixed(lsm6dsv_t *dev, int32_t gyr_f25[3]);

/** Accel-only raw → g F27 (clears XLDA). FLASH_ZW. */
int lsm6dsv_read_acc_fixed(lsm6dsv_t *dev, int32_t acc_g_f27[3]);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSV_H */
