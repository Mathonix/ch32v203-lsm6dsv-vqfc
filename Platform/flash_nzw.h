/**
 * Place cold / init-only code and strings into non-zero-wait Flash (FLASH_NZW).
 *
 * Linker: Startup/link.ld maps .text_nzw / .rodata_nzw → FLASH_NZW @ 0x8000.
 * Prefer for: platform_init, sensor init, VQF init, printf format paths,
 * euler helpers, unused 9D/mag helpers, etc.
 *
 * Hot path (vqf_sample_step, updateGyr/updateAcc + full callee graph, I2C read,
 * SysTick helpers) must use FLASH_ZW / default sections claimed by .text_zw
 * so addresses stay < 0x8000 (R0WAIT).
 */
#ifndef FLASH_NZW_H
#define FLASH_NZW_H

#define FLASH_NZW          __attribute__((section(".text_nzw")))
#define FLASH_NZW_RODATA   __attribute__((section(".rodata_nzw")))

#endif /* FLASH_NZW_H */
