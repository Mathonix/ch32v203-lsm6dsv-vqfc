/**
 * Mahony 6DOF attitude observer (Mahony et al.).
 * Section attributes place code in ALGO_RAM (VMA) / FLASH_NZW (LMA).
 *
 * Default gains: Kp=2.0, Ki=0.0 (integral optional; disabled to avoid drift
 * coupling without mag). Units: gyr rad/s, acc m/s².
 */
#include "mahony.h"
#include "fusion.h"

#include <math.h>

#define ALGO_TEXT __attribute__((section(".algo_text.mahony")))

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integral_fb_x, integral_fb_y, integral_fb_z;
static float two_kp = 2.0f * 1.0f;
static float two_ki = 0.0f;

ALGO_TEXT void mahony_set_gains(float kp, float ki)
{
    two_kp = 2.0f * kp;
    two_ki = 2.0f * ki;
}

ALGO_TEXT void mahony_init(float sample_hz)
{
    (void)sample_hz;
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    integral_fb_x = integral_fb_y = integral_fb_z = 0.0f;
    two_kp = 2.0f * 1.0f;
    two_ki = 0.0f;
}

ALGO_TEXT void mahony_update(const float gyr[3], const float acc[3], float dt)
{
    float gx = gyr[0], gy = gyr[1], gz = gyr[2];
    float ax = acc[0], ay = acc[1], az = acc[2];

    float recip_norm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* Compute feedback only if accelerometer measurement valid */
    float anorm2 = ax * ax + ay * ay + az * az;
    if (anorm2 > 1.0e-6f) {
        recip_norm = 1.0f / sqrtf(anorm2);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* Estimated gravity direction (from quaternion) */
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        /* Error = cross(estimated, measured) */
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if (two_ki > 0.0f) {
            integral_fb_x += two_ki * halfex * dt;
            integral_fb_y += two_ki * halfey * dt;
            integral_fb_z += two_ki * halfez * dt;
            gx += integral_fb_x;
            gy += integral_fb_y;
            gz += integral_fb_z;
        } else {
            integral_fb_x = integral_fb_y = integral_fb_z = 0.0f;
        }

        gx += two_kp * halfex;
        gy += two_kp * halfey;
        gz += two_kp * halfez;
    }

    /* Integrate rate of change of quaternion (0.5 * q ⊗ ω) */
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recip_norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;
}

ALGO_TEXT void mahony_get_quat(float q[4])
{
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

/* Vtable lives in normal NZW/rodata; pointers hold ALGO_RAM VMAs. */
const fusion_algo_t fusion_algo_mahony = {
    .init = mahony_init,
    .update = mahony_update,
    .get_quat = mahony_get_quat,
};
