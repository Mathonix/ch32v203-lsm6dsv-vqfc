/**
 * Place high-rate sample / VQF hot-path code into HOT_RAM (SRAM execute).
 *
 * Linker: Startup/link.ld maps .text.hot_ram → HOT_RAM @ 0x20000000
 * with LMA in zero-wait FLASH for boot memcpy (handle_reset).
 * Use for: 1 kHz IMU→VQF-fxp loop body and callees (vqfx_*, SPI Q16 read,
 * UART A5 5B / CAN i16 TX, platform_millis).
 *
 * Cold/init/printf stay in FLASH_NZW via flash_nzw.h.
 * Int64 div helpers (__divdi3 family) stay in .text_zw (ZW Flash, Option B).
 */
#ifndef FLASH_ZW_H
#define FLASH_ZW_H

#define FLASH_ZW __attribute__((section(".text.hot_ram")))

#endif /* FLASH_ZW_H */
