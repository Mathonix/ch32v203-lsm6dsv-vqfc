#include "ist8310.h"

int ist8310_init(ist8310_t *dev)
{
    if (dev == 0) {
        return -1;
    }
    dev->addr_7bit = IST8310_I2C_ADDR_7BIT;
    return -1; /* not implemented — Mag left off (see README) */
}

int ist8310_read_mag_f18(ist8310_t *dev, int32_t mag_uT_f18[3])
{
    (void)dev;
    if (mag_uT_f18) {
        mag_uT_f18[0] = mag_uT_f18[1] = mag_uT_f18[2] = 0;
    }
    return -1;
}
