/**
 * Place cold / init-only code and strings into non-zero-wait Flash (FLASH_NZW).
 *
 * Linker: Startup/link.ld maps .text_nzw / .rodata_nzw → FLASH_NZW @ 0x8000.
 * Prefer for: SystemInit is kept in zero-wait; use FLASH_NZW for platform_init,
 * sensor init, VQF init, printf format paths, unused 9D helpers, etc.
 *
 * Hot path (updateGyr/updateAcc/getQuat6D, I2C read, SysTick helpers) should
 * remain in default .text so the linker can fill residual R0WAIT first.
 */
#ifndef FLASH_NZW_H
#define FLASH_NZW_H

#define FLASH_NZW          __attribute__((section(".text_nzw")))
#define FLASH_NZW_RODATA   __attribute__((section(".rodata_nzw")))

#endif /* FLASH_NZW_H */
