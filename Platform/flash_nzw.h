/**
 * Place cold / init-only code and strings into non-zero-wait Flash (FLASH_NZW).
 *
 * Linker: Startup/link.ld maps .text_nzw / .rodata_nzw → FLASH_NZW @ 0x8000.
 * Prefer for: platform_init, sensor init, VQF init, printf format paths,
 * unused 9D/mag helpers, etc.
 *
 * Hot path (fusion_run_1khz_fxp, vqfx_*, SPI Q16 read, UART/CAN i16) must use
 * FLASH_ZW → .text.hot_ram (HOT_RAM SRAM execute after boot copy).
 */
#ifndef FLASH_NZW_H
#define FLASH_NZW_H

#define FLASH_NZW          __attribute__((section(".text_nzw")))
#define FLASH_NZW_RODATA   __attribute__((section(".rodata_nzw")))

#endif /* FLASH_NZW_H */
