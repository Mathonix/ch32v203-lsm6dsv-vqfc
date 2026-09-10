/**
 * 6DOF complementary filter on quaternion.
 * Gyro predicts; accelerometer corrects tilt (no yaw absolute reference).
 * Code in ALGO_RAM for SRAM execute when selected.
 */
#include "complementary.h"
#include "fusion.h"

#include <math.h>

#define ALGO_TEXT __attribute__((section(".algo_text.comp")))

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float alpha = 0.02f; /* accel correction weight per sample */

ALGO_TEXT void comp_set_alpha(float a)
{
    if (a < 0.0f) {
        a = 0.0f;
    } else if (a > 1.0f) {
        a = 1.0f;
    }
    alpha = a;
}

ALGO_TEXT void comp_init(float sample_hz)
{
    (void)sample_hz;
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    alpha = 0.02f;
}

ALGO_TEXT static void quat_normalize(void)
{
    float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (n > 1.0e-8f) {
        float inv = 1.0f / n;
        q0 *= inv;
        q1 *= inv;
        q2 *= inv;
        q3 *= inv;
    } else {
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
    }
}

ALGO_TEXT void comp_update(const float gyr[3], const float acc[3], float dt)
{
    float gx = gyr[0], gy = gyr[1], gz = gyr[2];

    /* Gyro integration */
    float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;
    quat_normalize();

    float ax = acc[0], ay = acc[1], az = acc[2];
    float an2 = ax * ax + ay * ay + az * az;
    if (an2 < 1.0e-6f) {
        return;
    }
    float inva = 1.0f / sqrtf(an2);
    ax *= inva;
    ay *= inva;
    az *= inva;

    /* Estimated gravity from current quat (body frame) */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* Error = measured × estimated (small-angle correction axis) */
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    /* Apply proportional correction as virtual gyro (scaled by alpha) */
    float corr = alpha;
    gx = ex * corr;
    gy = ey * corr;
    gz = ez * corr;
    dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;
    quat_normalize();
}

ALGO_TEXT void comp_get_quat(float q[4])
{
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

const fusion_algo_t fusion_algo_comp = {
    .init = comp_init,
    .update = comp_update,
    .get_quat = comp_get_quat,
};
