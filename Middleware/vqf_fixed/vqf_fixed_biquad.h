#ifndef VQF_FIXED_BIQUAD_H
#define VQF_FIXED_BIQUAD_H

#include <stdint.h>

typedef struct {
    int32_t x1, x2, y1, y2;
    int64_t residue;
    uint16_t init_count;
    uint8_t initialized;
    uint8_t _pad;
} vqf_biquad_state_t; /* 28 bytes */

typedef struct {
    int32_t b0, b1, b2, a1, a2;
} vqf_biquad_coeff_f30_t;

void vqf_biquad_reset(vqf_biquad_state_t *s);
int32_t vqf_biquad_step(const vqf_biquad_coeff_f30_t *c, vqf_biquad_state_t *s,
                        int32_t x, uint32_t init_samples);

#endif
