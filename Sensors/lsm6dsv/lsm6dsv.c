#include "lsm6dsv.h"
#include "platform.h"
#include "flash_nzw.h"
#include "flash_zw.h"

/* Register map (ST lsm6dsv-pid / DS13476) */
#define REG_WHO_AM_I   0x0Fu
#define REG_CTRL1      0x10u
#define REG_CTRL2      0x11u
#define REG_CTRL3      0x12u
#define REG_CTRL6      0x15u
#define REG_CTRL8      0x17u
#define REG_HAODR_CFG  0x62u
#define REG_OUTX_L_G   0x22u

#define CTRL3_SW_RESET (1u << 0)
#define CTRL3_IF_INC   (1u << 2)
#define CTRL3_BDU      (1u << 6)

/*
 * True 1 kHz via high-accuracy ODR mode (HAODR):
 *   HAODR_CFG.HAODR_SEL = 1 → ODR code 0x9 maps to 1000 Hz (not 960).
 *   CTRL1/CTRL2: OP_MODE = 001 (HAODR) in [6:4], ODR = 0x9 in [3:0]
 *     → register value 0x19 (ST LSM6DSV_ODR_HA01_AT_1000Hz).
 * Nominal rate used by VQF: LSM6DSV_ODR_HZ = 1000.
 */
#define HAODR_SEL_1000     0x01u /* HAODR_SEL_[1:0] = 01 */
#define OP_MODE_HAODR      0x10u /* OP_MODE_[2:0] = 001 << 4 */
#define ODR_CODE_1000      0x09u
#define CTRL_HAODR_1000HZ  (uint8_t)(OP_MODE_HAODR | ODR_CODE_1000) /* 0x19 */

#define FS_G_2000DPS   0x04u /* CTRL6[3:0] */
#define FS_XL_4G       0x01u /* CTRL8[1:0] */

/* Sensitivities from ST conversion helpers */
#define ACC_MG_PER_LSB     0.122f
#define GYR_MDPS_PER_LSB   70.0f
#define G_TO_MPS2          9.80665f
#define DEG2RAD            0.017453292519943295f

/* SPI: write addr MSB=0; read addr MSB=1 (OR 0x80). Mode 3 via platform SPI1. */

FLASH_ZW static int spi_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };
    platform_lsm_cs(1);
    int rc = platform_spi_xfer(tx, 0, 2);
    platform_lsm_cs(0);
    return rc;
}

FLASH_ZW static int spi_read_regs(uint8_t reg, uint8_t *buf, uint16_t n)
{
    uint8_t addr = (uint8_t)(reg | 0x80u);
    platform_lsm_cs(1);
    int rc = platform_spi_xfer(&addr, 0, 1);
    if (rc == 0) {
        rc = platform_spi_xfer(0, buf, n);
    }
    platform_lsm_cs(0);
    return rc;
}

static int soft_reset(lsm6dsv_t *dev)
{
    (void)dev;
    if (spi_write_reg(REG_CTRL1, 0x00) != 0) {
        return -1;
    }
    if (spi_write_reg(REG_CTRL2, 0x00) != 0) {
        return -2;
    }
    if (spi_write_reg(REG_CTRL3, CTRL3_SW_RESET) != 0) {
        return -3;
    }
    for (int i = 0; i < 20; i++) {
        uint8_t c3 = 0xFFu;
        platform_delay_ms(1);
        if (spi_read_regs(REG_CTRL3, &c3, 1) != 0) {
            return -4;
        }
        if ((c3 & CTRL3_SW_RESET) == 0u) {
            return 0;
        }
    }
    return -5;
}

int lsm6dsv_whoami(lsm6dsv_t *dev, uint8_t *id)
{
    if (dev == 0 || id == 0) {
        return -1;
    }
    return spi_read_regs(REG_WHO_AM_I, id, 1);
}

FLASH_NZW int lsm6dsv_init(lsm6dsv_t *dev)
{
    if (dev == 0) {
        return -1;
    }
    dev->unused = 0;

    platform_delay_ms(10);

    if (soft_reset(dev) != 0) {
        return -10;
    }
    platform_delay_ms(10);

    uint8_t who = 0;
    if (lsm6dsv_whoami(dev, &who) != 0) {
        return -11;
    }
    if (who != LSM6DSV_WHO_AM_I_VALUE) {
        return -100;
    }

    if (spi_write_reg(REG_CTRL3, (uint8_t)(CTRL3_BDU | CTRL3_IF_INC)) != 0) {
        return -12;
    }
    if (spi_write_reg(REG_CTRL6, FS_G_2000DPS) != 0) {
        return -13;
    }
    if (spi_write_reg(REG_CTRL8, FS_XL_4G) != 0) {
        return -14;
    }
    /* Select HAODR table so ODR code 0x9 is true 1000 Hz (not 960). */
    if (spi_write_reg(REG_HAODR_CFG, HAODR_SEL_1000) != 0) {
        return -17;
    }
    if (spi_write_reg(REG_CTRL1, CTRL_HAODR_1000HZ) != 0) {
        return -15;
    }
    if (spi_write_reg(REG_CTRL2, CTRL_HAODR_1000HZ) != 0) {
        return -16;
    }

    platform_delay_ms(20);
    return 0;
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

FLASH_ZW int lsm6dsv_read_acc_gyr(lsm6dsv_t *dev, float acc_mps2[3], float gyr_rads[3])
{
    uint8_t raw[12];
    if (dev == 0 || acc_mps2 == 0 || gyr_rads == 0) {
        return -1;
    }
    if (spi_read_regs(REG_OUTX_L_G, raw, 12) != 0) {
        return -2;
    }

    int16_t gx = le16(&raw[0]);
    int16_t gy = le16(&raw[2]);
    int16_t gz = le16(&raw[4]);
    int16_t ax = le16(&raw[6]);
    int16_t ay = le16(&raw[8]);
    int16_t az = le16(&raw[10]);

    const float acc_scale = (ACC_MG_PER_LSB * 0.001f) * G_TO_MPS2;
    const float gyr_scale = (GYR_MDPS_PER_LSB * 0.001f) * DEG2RAD;

    acc_mps2[0] = (float)ax * acc_scale;
    acc_mps2[1] = (float)ay * acc_scale;
    acc_mps2[2] = (float)az * acc_scale;

    gyr_rads[0] = (float)gx * gyr_scale;
    gyr_rads[1] = (float)gy * gyr_scale;
    gyr_rads[2] = (float)gz * gyr_scale;
    return 0;
}
