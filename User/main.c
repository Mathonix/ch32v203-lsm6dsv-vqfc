/**
 * CH32V203G6U6 + LSM6DSV + VQFC demo
 * - I2C1: PB6=SCL, PB7=SDA, LSM6DSV @ 0x6A (SA0=GND)
 * - USART1: PA9=TX @ 115200 for printf debug
 */
#include "platform.h"
#include "lsm6dsv.h"
#include "vqfc.h"

#include <stdint.h>

int main(void)
{
    platform_init();
    platform_uart_printf("\nCH32V203 + LSM6DSV + VQFC\n");
    platform_uart_printf("I2C1 PB6/PB7, USART1 PA9, IMU addr 0x6A\n");

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
    platform_uart_printf("LSM6DSV OK (WHO_AM_I=0x%02X)\n", LSM6DSV_WHO_AM_I_VALUE);

    vqfc_t filter;
    vqfc_init(&filter);

    uint32_t last_ms = platform_millis();
    uint32_t last_print = last_ms;
    const float dt_nom = 1.0f / 120.0f;

    while (1) {
        uint32_t now = platform_millis();
        float dt = (float)(now - last_ms) * 0.001f;
        if (dt < 0.001f) {
            continue; /* wait ~1 ms */
        }
        last_ms = now;
        if (dt > 0.05f) {
            dt = dt_nom;
        }

        float acc[3], gyr[3];
        if (lsm6dsv_read_acc_gyr(&imu, acc, gyr) != 0) {
            platform_uart_printf("IMU read error\n");
            platform_delay_ms(50);
            continue;
        }

        vqfc_update(&filter, gyr, acc, dt);

        if ((uint32_t)(now - last_print) >= 100u) {
            last_print = now;
            float q[4];
            float roll, pitch, yaw;
            vqfc_get_quat(&filter, q);
            vqfc_get_euler_deg(&filter, &roll, &pitch, &yaw);
            platform_uart_printf(
                "q=%.4f,%.4f,%.4f,%.4f  rpy=%.1f,%.1f,%.1f\n",
                (double)q[0], (double)q[1], (double)q[2], (double)q[3],
                (double)roll, (double)pitch, (double)yaw);
        }
    }
}
