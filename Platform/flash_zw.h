/**
 * Place high-rate sample / VQF hot-path code into HOT_RAM (SRAM execute).
 *
 * Linker: Startup/link.ld maps .text.hot_ram → HOT_RAM @ 0x20000000
 * with LMA in zero-wait FLASH for boot memcpy (handle_reset).
 * Use for: 4 kHz gyro / 1 kHz acc VQF fixed loop body and callees,
 * SPI fixed read, UART A5 5B / CAN i16 TX, platform_millis.
 *
 * Cold/init/printf stay in FLASH_NZW via flash_nzw.h.
 * Int64 div helpers (__divdi3 family) stay in .text_zw (ZW Flash, Option B).
 *
 * Host replay: define VQF_FIXED_HOST before include to neutralize attributes.
 */
#ifndef FLASH_ZW_H
#define FLASH_ZW_H

#ifdef VQF_FIXED_HOST
#define FLASH_ZW
#define FLASH_ZW_CODE
#else
#define FLASH_ZW __attribute__((section(".text.hot_ram")))
/** Zero-wait Flash resident (not SRAM). For helpers too large for HOT_RAM. */
#define FLASH_ZW_CODE __attribute__((section(".text.zw_vqf")))
#endif

#endif /* FLASH_ZW_H */
