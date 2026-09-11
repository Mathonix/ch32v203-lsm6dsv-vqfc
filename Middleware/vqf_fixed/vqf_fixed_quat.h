#ifndef VQF_FIXED_QUAT_H
#define VQF_FIXED_QUAT_H

#include <stdint.h>

void vqf_quat_mul_f30(const int32_t a[4], const int32_t b[4], int32_t o[4]);
void vqf_quat_normalize_f30(int32_t q[4]);
void vqf_quat_normalize_f30_if_needed(int32_t q[4]);
void vqf_quat_to_rotmat_f30(const int32_t q[4], int32_t R[9]);
void vqf_rotmat_vec(const int32_t R[9], const int32_t v[3], int32_t out[3]);
void vqf_quat_rotate_vec(const int32_t q[4], const int32_t v[3], int32_t out[3]);
/** Small-angle δq from v=0.5*Ts*w (v in F30): 4th-order cos/sinc poly. */
void vqf_quat_delta_from_v_f30(const int32_t v30[3], int32_t dq[4]);

#endif
