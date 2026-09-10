/**
 * Persistent algorithm-select flag in non-zero-wait Flash.
 *
 * Layout (16 bytes) at ALGO_CFG_ADDR_CPU / ALGO_CFG_ADDR_FPEC:
 *   magic    u32  0x414C474F ('ALGO')
 *   version  u32  1
 *   algo_id  u32  0=VQF, 1=Mahony, 2=complementary
 *   checksum u32  magic ^ version ^ algo_id
 *
 * LMA: last 4 KB page of CodeFlash reserved by link.ld
 *   FLASH_NZW shortened by 4K; ALGO_CFG @ 0x00037000 (CPU map).
 * FPEC programming uses the 0x08000000 alias (WCH convention).
 *
 * Change at runtime: UART line "SETALGO n" (n=0..2) then reboot, or
 *   python3 scripts/setalgo.py --id N build/firmware.bin
 *   (patches the bin image before flash), or openocd mww on FPEC addr.
 */
#ifndef ALGO_CFG_H
#define ALGO_CFG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ALGO_CFG_MAGIC          0x414C474Fu /* 'ALGO' */
#define ALGO_CFG_VERSION        1u

/** CPU / linker-visible address (FLASH base 0x00000000 in this project). */
#define ALGO_CFG_ADDR_CPU       0x00037000u
/** WCH FPEC program/erase address (Flash alias @ 0x08000000). */
#define ALGO_CFG_ADDR_FPEC      0x08037000u
/** Standard erase granularity for this slot. */
#define ALGO_CFG_PAGE_SIZE      0x1000u

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t algo_id;
    uint32_t checksum;
} algo_cfg_t;

/** Read mapped slot; returns 0 and fills *out if magic/version/checksum OK. */
int algo_cfg_read(algo_cfg_t *out);

/**
 * Erase 4 KB page and program a valid cfg for algo_id.
 * Returns 0 on success, negative on failure.
 * Runs from NZW (cold path); takes several ms.
 */
int algo_cfg_write(uint32_t algo_id);

/** Checksum helper (exported for host tools / tests). */
static inline uint32_t algo_cfg_checksum(uint32_t magic, uint32_t version, uint32_t algo_id)
{
    return magic ^ version ^ algo_id;
}

#ifdef __cplusplus
}
#endif

#endif /* ALGO_CFG_H */
