#include "vqfc.h"

#include <math.h>

#ifndef VQFC_PI
#define VQFC_PI 3.14159265358979323846f
#endif

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static void quat_normalize(float q[4])
{
    float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (n2 < 1.0e-20f) {
        q[0] = 1.f;
        q[1] = q[2] = q[3] = 0.f;
        return;
    }
    float inv = 1.0f / sqrtf(n2);
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
}

void vqfc_init(vqfc_t *f)
{
    if (f == 0) {
        return;
    }
    f->q[0] = 1.f;
    f->q[1] = 0.f;
    f->q[2] = 0.f;
    f->q[3] = 0.f;
    f->gyr_bias[0] = f->gyr_bias[1] = f->gyr_bias[2] = 0.f;
    /* Defaults tuned for ~100–200 Hz IMU demos */
    f->k_acc = 1.0f;     /* rad/s toward measured gravity */
    f->k_bias = 0.01f;   /* slow bias learning */
    f->acc_norm_min = 0.8f * 9.80665f;
    f->acc_norm_max = 1.2f * 9.80665f;
}

void vqfc_update(vqfc_t *f, const float gyr[3], const float acc[3], float dt)
{
    if (f == 0 || gyr == 0 || acc == 0 || dt <= 0.f) {
        return;
    }
    if (dt > 0.05f) {
        dt = 0.05f; /* clamp after stalls */
    }

    float wx = gyr[0] - f->gyr_bias[0];
    float wy = gyr[1] - f->gyr_bias[1];
    float wz = gyr[2] - f->gyr_bias[2];

    float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    int acc_ok = (an > f->acc_norm_min) && (an < f->acc_norm_max);

    if (acc_ok) {
        float ax = acc[0] / an;
        float ay = acc[1] / an;
        float az = acc[2] / an;

        /* Predicted gravity in body frame: R^T * [0,0,1] from quaternion */
        float qw = f->q[0], qx = f->q[1], qy = f->q[2], qz = f->q[3];
        float gx = 2.f * (qx * qz - qw * qy);
        float gy = 2.f * (qy * qz + qw * qx);
        float gz = qw * qw - qx * qx - qy * qy + qz * qz;

        /* Error = acc × g_pred (small-angle tilt error in body axes) */
        float ex = ay * gz - az * gy;
        float ey = az * gx - ax * gz;
        float ez = ax * gy - ay * gx;

        wx += f->k_acc * ex;
        wy += f->k_acc * ey;
        wz += f->k_acc * ez;

        f->gyr_bias[0] -= f->k_bias * ex * dt;
        f->gyr_bias[1] -= f->k_bias * ey * dt;
        f->gyr_bias[2] -= f->k_bias * ez * dt;
        f->gyr_bias[0] = clampf(f->gyr_bias[0], -0.2f, 0.2f);
        f->gyr_bias[1] = clampf(f->gyr_bias[1], -0.2f, 0.2f);
        f->gyr_bias[2] = clampf(f->gyr_bias[2], -0.2f, 0.2f);
    }

    /* Quaternion integrate: q_dot = 0.5 * q ⊗ [0,w] */
    float qw = f->q[0], qx = f->q[1], qy = f->q[2], qz = f->q[3];
    float dq0 = 0.5f * (-qx * wx - qy * wy - qz * wz);
    float dq1 = 0.5f * (qw * wx + qy * wz - qz * wy);
    float dq2 = 0.5f * (qw * wy - qx * wz + qz * wx);
    float dq3 = 0.5f * (qw * wz + qx * wy - qy * wx);

    f->q[0] = qw + dq0 * dt;
    f->q[1] = qx + dq1 * dt;
    f->q[2] = qy + dq2 * dt;
    f->q[3] = qz + dq3 * dt;
    quat_normalize(f->q);
}

void vqfc_get_quat(const vqfc_t *f, float q[4])
{
    if (f == 0 || q == 0) {
        return;
    }
    q[0] = f->q[0];
    q[1] = f->q[1];
    q[2] = f->q[2];
    q[3] = f->q[3];
}

void vqfc_get_euler_deg(const vqfc_t *f, float *roll, float *pitch, float *yaw)
{
    if (f == 0) {
        return;
    }
    float w = f->q[0], x = f->q[1], y = f->q[2], z = f->q[3];

    float sinr_cosp = 2.f * (w * x + y * z);
    float cosr_cosp = 1.f - 2.f * (x * x + y * y);
    float r = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.f * (w * y - z * x);
    sinp = clampf(sinp, -1.f, 1.f);
    float p = asinf(sinp);

    float siny_cosp = 2.f * (w * z + x * y);
    float cosy_cosp = 1.f - 2.f * (y * y + z * z);
    float yv = atan2f(siny_cosp, cosy_cosp);

    const float rad2deg = 180.f / VQFC_PI;
    if (roll) {
        *roll = r * rad2deg;
    }
    if (pitch) {
        *pitch = p * rad2deg;
    }
    if (yaw) {
        *yaw = yv * rad2deg;
    }
}
