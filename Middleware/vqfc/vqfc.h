/**
 * VQFC — compact 6DOF quaternion attitude filter (VQF / complementary inspired).
 * Pure C, no malloc. Sized for ~10 KB SRAM MCUs.
 *
 * Gains (defaults):
 *   - gyr bias LP tau ~ few seconds via vqf-like slow correction (k_bias)
 *   - tilt correction gain k_acc ~ 0.5..2 rad/s toward accel gravity
 * Documented in README.
 */
#ifndef VQFC_H
#define VQFC_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q[4];       /* w,x,y,z */
    float gyr_bias[3];
    float k_acc;      /* tilt correction gain [1/s] */
    float k_bias;     /* gyro bias adaptation [1/s^2] */
    float acc_norm_min;
    float acc_norm_max;
} vqfc_t;

void vqfc_init(vqfc_t *f);
void vqfc_update(vqfc_t *f, const float gyr[3], const float acc[3], float dt);
void vqfc_get_quat(const vqfc_t *f, float q[4]);
void vqfc_get_euler_deg(const vqfc_t *f, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* VQFC_H */
