/**
 * Compact Mahony AHRS (6DOF, no magnetometer).
 * Tunable gains kp/ki. State in .bss; code+rodata in ALGO_RAM (SRAM execute).
 */
#ifndef MAHONY_H
#define MAHONY_H

#ifdef __cplusplus
extern "C" {
#endif

void mahony_init(float sample_hz);
void mahony_update(const float gyr[3], const float acc[3], float dt);
void mahony_get_quat(float q[4]);

/** Proportional / integral gains (defaults: 2*1.0, 2*0.0 — no windup for 6DOF). */
void mahony_set_gains(float kp, float ki);

#ifdef __cplusplus
}
#endif

#endif
