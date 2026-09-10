/**
 * Compact complementary filter (6DOF).
 * First-order lerp between gyro integration and accel tilt.
 */
#ifndef COMPLEMENTARY_H
#define COMPLEMENTARY_H

#ifdef __cplusplus
extern "C" {
#endif

void comp_init(float sample_hz);
void comp_update(const float gyr[3], const float acc[3], float dt);
void comp_get_quat(float q[4]);

/** Accel trust α in [0,1]; default 0.02 @ 1 kHz (~ tau ≈ 50 ms). */
void comp_set_alpha(float alpha);

#ifdef __cplusplus
}
#endif

#endif
