/**
 * Minimal LSM6DSV driver (I2C) for bare-metal firmware.
 * WHO_AM_I verified against ST PID / datasheet: 0x70.
 */
#ifndef LSM6DSV_H
#define LSM6DSV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** ST LSM6DSV device ID (WHO_AM_I @ 0x0F). */
#define LSM6DSV_WHO_AM_I_VALUE   0x70u

/** 7-bit I2C address: SA0/SDO = GND -> 0x6A; SA0 = Vdd_IO -> 0x6B. */
#define LSM6DSV_I2C_ADDR_SA0_L   0x6Au
#define LSM6DSV_I2C_ADDR_SA0_H   0x6Bu

/**
 * Configured XL+GY output data rate (Hz).
 * High-accuracy ODR (HAODR_SEL=1, ODR code 0x9) → true 1000 Hz
 * (standard mode without HAODR would be 960 Hz at the same ODR code).
 */
#define LSM6DSV_ODR_HZ           1000.0f

typedef struct {
    uint8_t addr7;
} lsm6dsv_t;

/**
 * Soft-reset, configure ODR 1000 Hz (HAODR), FS ±4 g / ±2000 dps, BDU+IF_INC.
 * @return 0 on success, negative on I2C/config error, -100 on WHO_AM_I mismatch
 */
int lsm6dsv_init(lsm6dsv_t *dev, uint8_t addr7);

/** Read WHO_AM_I register. */
int lsm6dsv_whoami(lsm6dsv_t *dev, uint8_t *id);

/**
 * Read accelerometer in m/s^2 (x,y,z) and gyroscope in rad/s (x,y,z).
 * Uses FS sensitivities: 0.122 mg/LSB (±4g), 70 mdps/LSB (±2000 dps).
 */
int lsm6dsv_read_acc_gyr(lsm6dsv_t *dev, float acc_mps2[3], float gyr_rads[3]);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSV_H */
