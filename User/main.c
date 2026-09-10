/**
 * CH32V203G6U6 + LSM6DSV + VQF-C (dusking1/vqf-c full VQF) demo
 * - I2C1: PB6=SCL, PB7=SDA, LSM6DSV @ 0x6A (SA0=GND)
 * - USART1: PA9=TX @ 921600 binary Euler @ 1 kHz (+ boot banner via printf)
 * - CAN1: PA11=RX, PA12=TX @ 1 Mbit, std ID 0x321, Euler millideg @ 1 kHz
 * - 6DOF only: no magnetometer; initVqf(..., magTs=5.0f), never call updateMag
 *
 * Units (VQF / Laidig–Seel): gyroscope rad/s, accelerometer m/s².
 * LSM6DSV driver already converts to those SI units.
 *
 * Sample rate: LSM6DSV HAODR true 1000 Hz → gyrTs = accTs = 1/1000 s.
 * Hot path (sample + Euler + UART binary + CAN TX) lives in zero-wait Flash
 * (.text.hot / .text_zw, addresses < 0x8000).
 */
#include "platform.h"
#include "flash_nzw.h"
#include "flash_zw.h"
#include "lsm6dsv.h"
#include "vqf.h"

#include <math.h>
#include <stdint.h>

#ifndef VQF_UART_ASCII_DEBUG
#define VQF_UART_ASCII_DEBUG 0
#endif

/** Quaternion (w,x,y,z) → roll/pitch/yaw in degrees (aerospace ZYX). ZW. */
FLASH_ZW static void quat_to_euler_deg(const float q[4], float *roll, float *pitch, float *yaw)
{
    const float rad2deg = 57.2957795f; /* 180/pi; avoid M_PI for newlib-nano */
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float sinr_cosp = 2.0f * (w * x + y * z);
    const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp) * rad2deg;

    float sinp = 2.0f * (w * y - z * x);
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }
    *pitch = asinf(sinp) * rad2deg;

    const float siny_cosp = 2.0f * (w * z + x * y);
    const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp) * rad2deg;
}

#if VQF_UART_ASCII_DEBUG
/** Optional low-rate ASCII (NZW) — off by default; printf cannot sustain 1 kHz. */
FLASH_NZW static void vqf_uart_print_status(const float q[4])
{
    float roll, pitch, yaw;
    quat_to_euler_deg(q, &roll, &pitch, &yaw);
    platform_uart_printf(
        "roll=%.2f pitch=%.2f yaw=%.2f | q=%.4f,%.4f,%.4f,%.4f\n",
        (double)roll, (double)pitch, (double)yaw,
        (double)q[0], (double)q[1], (double)q[2], (double)q[3]);
}
#endif

/**
 * 1 kHz hot body in zero-wait Flash: IMU read → updateGyr → updateAcc → getQuat6D.
 * Returns 0 on success, negative on IMU read error.
 */
FLASH_ZW int vqf_sample_step(lsm6dsv_t *imu, float q_out[4])
{
    float acc[3], gyr[3];
    if (lsm6dsv_read_acc_gyr(imu, acc, gyr) != 0) {
        return -1;
    }
    /* SI units from driver: gyr rad/s, acc m/s² — as required by VQF */
    updateGyr(gyr);
    updateAcc(acc);
    /* 6DOF: do not call updateMag */
    getQuat6D(q_out);
    return 0;
}

/**
 * Forever 1 kHz sample loop — entire function in ZW.
 * Each sample: Euler → UART binary packet + CAN frame.
 */
FLASH_ZW void vqf_run_1khz(lsm6dsv_t *imu)
{
    uint32_t last_ms = platform_millis();
    const uint32_t sample_period_ms = 1u; /* 1000 Hz wall-clock cadence */
    float q[4];
    uint16_t seq = 0u;
#if VQF_UART_ASCII_DEBUG
    uint32_t last_print = last_ms;
    const uint32_t print_period_ms = 50u;
#endif

    while (1) {
        uint32_t now = platform_millis();
        if ((uint32_t)(now - last_ms) < sample_period_ms) {
            continue;
        }
        last_ms = now;

        if (vqf_sample_step(imu, q) != 0) {
            platform_delay_ms(1);
            continue;
        }

        float roll, pitch, yaw;
        quat_to_euler_deg(q, &roll, &pitch, &yaw);
        (void)platform_uart_send_euler_bin(seq, roll, pitch, yaw);
        (void)platform_can_send_euler(seq, roll, pitch, yaw);
        seq++;

#if VQF_UART_ASCII_DEBUG
        if ((uint32_t)(now - last_print) >= print_period_ms) {
            last_print = now;
            vqf_uart_print_status(q);
        }
#endif
    }
}

FLASH_NZW int main(void)
{
    platform_init();
    platform_uart_printf("\nCH32V203 + LSM6DSV + VQF-C (6DOF, 1 kHz ZW)\n");
    platform_uart_printf("I2C1 PB6/PB7, USART1 PA9 @ %u baud binary Euler\n",
                         (unsigned)PLATFORM_UART_BAUD);
    platform_uart_printf("CAN1 PA11/PA12 @ %u bit/s, std ID 0x%03X (transceiver req.)\n",
                         (unsigned)PLATFORM_CAN_BITRATE, (unsigned)PLATFORM_CAN_STD_ID);
    platform_uart_printf("USBD off (CAN shares SRAM). Packet: see README.\n");

    lsm6dsv_t imu;
    int rc = lsm6dsv_init(&imu, LSM6DSV_I2C_ADDR_SA0_L);
    if (rc == -100) {
        uint8_t who = 0;
        (void)lsm6dsv_whoami(&imu, &who);
        platform_uart_printf("WHO_AM_I fail: got 0x%02X expect 0x%02X — halt\n",
                             who, LSM6DSV_WHO_AM_I_VALUE);
        while (1) {
        }
    }
    if (rc != 0) {
        platform_uart_printf("LSM6DSV init error %d — halt\n", rc);
        while (1) {
        }
    }
    platform_uart_printf("LSM6DSV OK (WHO_AM_I=0x%02X, ODR=%.0f Hz HAODR)\n",
                         LSM6DSV_WHO_AM_I_VALUE, (double)LSM6DSV_ODR_HZ);

    /* Match LSM6DSV 1000 Hz HAODR; large magTs + no updateMag => 6DOF mode */
    const float gyrTs = 1.0f / LSM6DSV_ODR_HZ;
    const float accTs = 1.0f / LSM6DSV_ODR_HZ;
    const float magTs = 5.0f;
    initVqf(gyrTs, accTs, magTs);
    platform_uart_printf("VQF init: gyrTs=accTs=1/%.0f, magTs=5.0 (no mag)\n",
                         (double)LSM6DSV_ODR_HZ);
    platform_uart_printf("Hot path ZW: sample+euler+uart_bin+can_tx @ 1 kHz\n");

    /* Never return — 1 kHz loop executes from FLASH_ZW */
    vqf_run_1khz(&imu);
    return 0;
}
