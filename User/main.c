/**
 * CH32V203G6U6 + LSM6DSV + multi-algo fusion demo
 *
 * Schematic board MCU is AT32F423KCU7-4; this firmware targets CH32V203 with
 * matching net names where AF coincides (SPI). See README pin table.
 * - SPI1: PA4=LSM_CS (SW), PA5=SCK, PA6=MISO, PA7=MOSI — mode 3, WHO_AM_I=0x70
 * - PB0=LSM_INT1, PB1=LSM_INT2 (inputs, unused)
 * - USART2: PA2=TX, PA3=RX @ 921600 binary Euler @ 1 kHz (+ boot printf)
 *   (Schematic UART nets are PA0/PA1 on AT32; CH32 has no USART data AF there)
 * - CAN1 Remap1: PA11=RX, PA12=TX @ 1 Mbit, std ID 0x321
 *   (Schematic CAN nets are PA2/PA3 on AT32; CH32 cannot remap CAN there)
 * - Default algo: Full VQF fixed-point (vqf_fixed) 6D in HOT_RAM (SRAM)
 * - Alternates (Mahony / complementary): NZW LMA → ALGO_RAM, execute from SRAM
 *
 * Units: gyroscope rad/s, accelerometer m/s².
 * Sample rate: LSM6DSV HAODR true 1000 Hz.
 *
 * UART command (boot window ~1.5 s, or anytime before SETALGO finishes):
 *   SETALGO n\n   with n=0 VQF, 1 Mahony, 2 complementary — programs Flash flag,
 *   then soft-hint to reset (user power-cycles / NRST).
 */
#include "platform.h"
#include "flash_nzw.h"
#include "flash_zw.h"
#include "lsm6dsv.h"
#include "fusion.h"
#include "vqf_fixed.h"
#include "vqf_fxp.h" /* deprecated wrappers */
#include "algo_cfg.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef VQF_UART_ASCII_DEBUG
#define VQF_UART_ASCII_DEBUG 0
#endif

/** Quaternion (w,x,y,z) → roll/pitch/yaw in degrees (aerospace ZYX). ZW. */
FLASH_NZW static void quat_to_euler_deg(const float q[4], float *roll, float *pitch, float *yaw)
{
    const float rad2deg = 57.2957795f;
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
FLASH_NZW static void fusion_uart_print_status(const float q[4])
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
 * 1 kHz hot body in HOT_RAM (SRAM): IMU → vqfx → Euler i16 UART/CAN.
 * VQF callees in HOT_RAM; Mahony/Comp run from ALGO_RAM via function pointers.
 */
/**
 * Float sample step — used by Mahony / complementary (ALGO_RAM).
 * Not used when ALGO_VQF (fixed-point path below).
 */
FLASH_NZW int fusion_sample_step(lsm6dsv_t *imu, float q_out[4], float dt)
{
    float acc[3], gyr[3];
    if (lsm6dsv_read_acc_gyr(imu, acc, gyr) != 0) {
        return -1;
    }
    fusion_active->update(gyr, acc, dt);
    fusion_active->get_quat(q_out);
    return 0;
}

/** Clamp millideg to int16 for UART/CAN. */
FLASH_ZW static int16_t mdeg_to_i16(int32_t mdeg)
{
    if (mdeg > 32767) {
        return (int16_t)32767;
    }
    if (mdeg < -32768) {
        return (int16_t)-32768;
    }
    return (int16_t)mdeg;
}

/**
 * Full VQF fixed 1 kHz body (ALGO_VQF): raw LSM → F25/F27 → vqf_fixed → millideg.
 * Gyro update is rate-independent (structure ready for 4 kHz gyr / 1 kHz acc).
 * No soft-float / libm on this path.
 */
FLASH_ZW static void fusion_run_1khz_fxp(lsm6dsv_t *imu)
{
    uint32_t last_ms = platform_millis();
    const uint32_t sample_period_ms = 1u;
    uint16_t seq = 0u;
    int32_t acc_f27[3], gyr_f25[3];
    int32_t roll_m, pitch_m, yaw_m;

    while (1) {
        uint32_t now = platform_millis();
        if ((uint32_t)(now - last_ms) < sample_period_ms) {
            continue;
        }
        last_ms = now;

        if (lsm6dsv_read_acc_gyr_fixed(imu, acc_f27, gyr_f25) != 0) {
            platform_delay_ms(1);
            continue;
        }
        vqf_fixed_update_gyr_f25(gyr_f25);
        vqf_fixed_update_acc_f27(acc_f27);
        vqf_fixed_get_euler_mdeg(&roll_m, &pitch_m, &yaw_m);

        int16_t r = mdeg_to_i16(roll_m);
        int16_t p = mdeg_to_i16(pitch_m);
        int16_t y = mdeg_to_i16(yaw_m);
        (void)platform_uart_send_euler_i16(seq, r, p, y);
        (void)platform_can_send_euler_i16(seq, r, p, y);
        seq++;
    }
}

/** Mahony / complementary 1 kHz loop (float) — NZW; soft-float OK here. */
FLASH_NZW static void fusion_run_1khz_float(lsm6dsv_t *imu)
{
    uint32_t last_ms = platform_millis();
    const uint32_t sample_period_ms = 1u;
    const float dt = 1.0f / LSM6DSV_ODR_HZ;
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

        if (fusion_sample_step(imu, q, dt) != 0) {
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
            fusion_uart_print_status(q);
        }
#endif
    }
}

FLASH_ZW void fusion_run_1khz(lsm6dsv_t *imu)
{
    /* Default ALGO_VQF integer path only — never returns.
     * Mahony/Comp use fusion_run_1khz_float from main (NZW). */
    fusion_run_1khz_fxp(imu);
}

/** Parse "SETALGO n" from a line buffer; returns 0 and sets *id_out on success. */
FLASH_NZW static int parse_setalgo(const char *line, uint32_t *id_out)
{
    const char *p = line;
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (strncmp(p, "SETALGO", 7) != 0) {
        return -1;
    }
    p += 7;
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (*p < '0' || *p > '2' || (*(p + 1) != '\0' && *(p + 1) != '\r' && *(p + 1) != '\n' && *(p + 1) != ' ')) {
        return -2;
    }
    *id_out = (uint32_t)(*p - '0');
    return 0;
}

/**
 * Drain UART for up to wait_ms looking for a SETALGO line.
 * On success, programs Flash and prints reboot hint.
 */
FLASH_NZW static void fusion_poll_setalgo_window(uint32_t wait_ms)
{
    char buf[32];
    unsigned n = 0;
    uint32_t t0 = platform_millis();
    platform_uart_printf("SETALGO window %u ms (send SETALGO 0|1|2)\n", (unsigned)wait_ms);

    while ((uint32_t)(platform_millis() - t0) < wait_ms) {
        int c = platform_uart_getc_nonblock();
        if (c < 0) {
            continue;
        }
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            buf[n < sizeof(buf) ? n : sizeof(buf) - 1u] = '\0';
            uint32_t id = 0;
            if (n > 0 && parse_setalgo(buf, &id) == 0) {
                platform_uart_printf("Programming algo_id=%u ...\n", (unsigned)id);
                int rc = algo_cfg_write(id);
                if (rc == 0) {
                    platform_uart_printf("OK — reset MCU to apply (%s)\n",
                                         fusion_algo_name((algo_id_t)id));
                } else {
                    platform_uart_printf("Flash write failed (%d)\n", rc);
                }
            }
            n = 0;
            continue;
        }
        if (n + 1u < sizeof(buf)) {
            buf[n++] = (char)c;
        } else {
            n = 0;
        }
    }
}

FLASH_NZW int main(void)
{
    platform_init();
    platform_uart_printf("\nCH32V203 + LSM6DSV SPI + multi-algo fusion (6DOF, 1 kHz, VQF-fxp)\n");
    platform_uart_printf("Schematic MCU=AT32F423; this FW=CH32V203 (SPI nets match; UART/CAN AF differ)\n");
    platform_uart_printf("SPI1 PA4=CS PA5=SCK PA6=MISO PA7=MOSI mode3; INT1/2=PB0/PB1 unused\n");
    platform_uart_printf("USART2 PA2=TX PA3=RX @ %u (RM: no USART data AF on schematic PA0/PA1)\n",
                         (unsigned)PLATFORM_UART_BAUD);
    platform_uart_printf("CAN1 Remap1 PA11/PA12 @ %u ID 0x%03X (RM: no CAN AF on schematic PA2/PA3)\n",
                         (unsigned)PLATFORM_CAN_BITRATE, (unsigned)PLATFORM_CAN_STD_ID);

    algo_cfg_t cfg;
    int cfg_rc = algo_cfg_read(&cfg);
    fusion_boot_select();
    if (cfg_rc != 0) {
        platform_uart_printf("Algo flag: missing/invalid @ 0x%08X → default VQF (ZW)\n",
                             (unsigned)ALGO_CFG_ADDR_CPU);
    } else {
        platform_uart_printf("Algo flag: id=%u checksum OK @ 0x%08X\n",
                             (unsigned)cfg.algo_id, (unsigned)ALGO_CFG_ADDR_CPU);
    }
    platform_uart_printf("Selected: %s\n", fusion_algo_name(fusion_active_id));

    fusion_poll_setalgo_window(1500u);

    /* Re-read in case SETALGO wrote a new flag (takes effect next boot;
     * still re-select so a same-boot soft switch works after write + re-init). */
    fusion_boot_select();
    platform_uart_printf("Active after window: %s\n", fusion_algo_name(fusion_active_id));

    lsm6dsv_t imu;
    int rc = lsm6dsv_init(&imu);
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
    platform_uart_printf("LSM6DSV SPI OK (WHO_AM_I=0x%02X, ODR=%.0f Hz HAODR)\n",
                         LSM6DSV_WHO_AM_I_VALUE, (double)LSM6DSV_ODR_HZ);

    fusion_active->init(LSM6DSV_ODR_HZ);
    platform_uart_printf("Fusion init @ %.0f Hz\n", (double)LSM6DSV_ODR_HZ);
    if (fusion_active_id == ALGO_VQF) {
        platform_uart_printf("Hot path: VQF-fxp (Q30/Q16) in HOT_RAM; millideg UART A5 5B + CAN @ 1 kHz\n");
    } else {
        platform_uart_printf("Hot path: algo in ALGO_RAM (SRAM); I/O+euler in ZW @ 1 kHz\n");
    }

    if (fusion_active_id == ALGO_VQF) {
        fusion_run_1khz(&imu);
    } else {
        fusion_run_1khz_float(&imu);
    }
    return 0;
}
