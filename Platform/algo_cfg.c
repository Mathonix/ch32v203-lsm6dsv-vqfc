/**
 * Read / program the algo select flag in NZW Flash (standard 4 KB page mode).
 */
#include "algo_cfg.h"
#include "ch32v203_regs.h"
#include "flash_nzw.h"

#include <stddef.h>

FLASH_NZW static int flash_wait(uint32_t timeout)
{
    while ((FLASH_STATR & FLASH_STATR_BSY) != 0u) {
        if (timeout-- == 0u) {
            return -1;
        }
    }
    if ((FLASH_STATR & FLASH_STATR_WRPRTERR) != 0u) {
        FLASH_STATR = FLASH_STATR_WRPRTERR;
        return -2;
    }
    if ((FLASH_STATR & FLASH_STATR_EOP) != 0u) {
        FLASH_STATR = FLASH_STATR_EOP;
    }
    return 0;
}

FLASH_NZW static void flash_unlock(void)
{
    if ((FLASH_CTLR & FLASH_CTLR_LOCK) != 0u) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
}

FLASH_NZW static void flash_lock(void)
{
    FLASH_CTLR |= FLASH_CTLR_LOCK;
}

FLASH_NZW int algo_cfg_read(algo_cfg_t *out)
{
    if (out == NULL) {
        return -1;
    }
    const volatile algo_cfg_t *p = (const volatile algo_cfg_t *)(uintptr_t)ALGO_CFG_ADDR_CPU;
    algo_cfg_t tmp;
    tmp.magic = p->magic;
    tmp.version = p->version;
    tmp.algo_id = p->algo_id;
    tmp.checksum = p->checksum;

    if (tmp.magic != ALGO_CFG_MAGIC || tmp.version != ALGO_CFG_VERSION) {
        return -2;
    }
    if (tmp.checksum != algo_cfg_checksum(tmp.magic, tmp.version, tmp.algo_id)) {
        return -3;
    }
    if (tmp.algo_id > 2u) {
        return -4;
    }
    *out = tmp;
    return 0;
}

FLASH_NZW static int flash_program_word(uint32_t addr, uint32_t data)
{
    int st = flash_wait(0x100000u);
    if (st != 0) {
        return st;
    }
    FLASH_CTLR |= FLASH_CTLR_PG;
    *(volatile uint16_t *)(uintptr_t)addr = (uint16_t)(data & 0xFFFFu);
    st = flash_wait(0x100000u);
    if (st != 0) {
        FLASH_CTLR &= ~FLASH_CTLR_PG;
        return st;
    }
    *(volatile uint16_t *)(uintptr_t)(addr + 2u) = (uint16_t)(data >> 16);
    st = flash_wait(0x100000u);
    FLASH_CTLR &= ~FLASH_CTLR_PG;
    return st;
}

FLASH_NZW int algo_cfg_write(uint32_t algo_id)
{
    if (algo_id > 2u) {
        return -1;
    }

    algo_cfg_t cfg;
    cfg.magic = ALGO_CFG_MAGIC;
    cfg.version = ALGO_CFG_VERSION;
    cfg.algo_id = algo_id;
    cfg.checksum = algo_cfg_checksum(cfg.magic, cfg.version, cfg.algo_id);

    flash_unlock();
    FLASH_STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;

    if (flash_wait(0x100000u) != 0) {
        flash_lock();
        return -2;
    }
    FLASH_CTLR |= FLASH_CTLR_PER;
    FLASH_ADDR = ALGO_CFG_ADDR_FPEC;
    FLASH_CTLR |= FLASH_CTLR_STRT;
    if (flash_wait(0x10000000u) != 0) {
        FLASH_CTLR &= ~FLASH_CTLR_PER;
        flash_lock();
        return -3;
    }
    FLASH_CTLR &= ~FLASH_CTLR_PER;

    const uint32_t *words = (const uint32_t *)&cfg;
    for (unsigned i = 0; i < (sizeof(cfg) / 4u); i++) {
        if (flash_program_word(ALGO_CFG_ADDR_FPEC + i * 4u, words[i]) != 0) {
            flash_lock();
            return -4;
        }
    }

    flash_lock();

    algo_cfg_t check;
    if (algo_cfg_read(&check) != 0 || check.algo_id != algo_id) {
        return -5;
    }
    return 0;
}
