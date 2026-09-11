/**
 * IST8310 magnetometer stub (schematic I2C on PB6/PB7).
 *
 * Not linked into firmware while VQF_FIXED_ENABLE_MAG=0.
 * Schematic net MAG_SCL/SDA = PB6/PB7 (CH32 I2C1 default AF).
 * Enable path later: implement I2C read → uT F18 → vqf_fixed_update_mag_f18().
 */
#ifndef IST8310_H
#define IST8310_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IST8310_I2C_ADDR_7BIT  0x0Eu /* common default; confirm on board */

typedef struct {
    uint8_t addr_7bit;
} ist8310_t;

/** Stub: returns -1 (not implemented). */
int ist8310_init(ist8310_t *dev);

/** Stub: mag in microtesla → F18 (×2^18). Returns -1. */
int ist8310_read_mag_f18(ist8310_t *dev, int32_t mag_uT_f18[3]);

#ifdef __cplusplus
}
#endif

#endif /* IST8310_H */
