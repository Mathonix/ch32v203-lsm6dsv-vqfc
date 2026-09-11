#include "vqf_fixed_biquad.h"
#include "vqf_fixed_math.h"
#include "flash_zw.h"
#include <string.h>

FLASH_ZW_CODE void vqf_biquad_reset(vqf_biquad_state_t *s)
{
    memset(s, 0, sizeof(*s));
}

FLASH_ZW_CODE int32_t vqf_biquad_step(const vqf_biquad_coeff_f30_t *c, vqf_biquad_state_t *s,
                                 int32_t x, uint32_t init_samples)
{
    if (!s->initialized) {
        /* Running mean without int64 sum: y += (x-y)/(n+1) */
        uint32_t n = s->init_count;
        if (n == 0u) {
            s->y1 = x;
        } else {
            s->y1 += (int32_t)(((int64_t)x - s->y1) / (int64_t)(n + 1u));
        }
        s->x1 = s->x2 = s->y2 = s->y1;
        s->residue = 0;
        s->init_count = (uint16_t)(n + 1u);
        if ((uint32_t)s->init_count >= init_samples) {
            s->initialized = 1u;
        }
        return s->y1;
    }

    int64_t acc = (int64_t)c->b0 * x + (int64_t)c->b1 * s->x1 + (int64_t)c->b2 * s->x2 -
                  (int64_t)c->a1 * s->y1 - (int64_t)c->a2 * s->y2 + s->residue;
    int32_t y = vqf_round_asr(acc, 30u);
    s->residue = acc - ((int64_t)y << 30);
    s->x2 = s->x1;
    s->x1 = x;
    s->y2 = s->y1;
    s->y1 = y;
    return y;
}
