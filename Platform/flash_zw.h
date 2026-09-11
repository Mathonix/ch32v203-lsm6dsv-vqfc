/**
 * Place high-rate sample / VQF hot-path code into zero-wait Flash (FLASH / R0WAIT).
 *
 * Linker: Startup/link.ld maps .text.hot → .text_zw → FLASH @ < 0x8000.
 * Use for: 1 kHz IMU→VQF loop body (fusion_sample_step), and any app helpers that
 * must execute from R0WAIT alongside updateGyr / updateAcc / SPI.
 *
 * Cold/init/printf stay in FLASH_NZW via flash_nzw.h.
 */
#ifndef FLASH_ZW_H
#define FLASH_ZW_H

#define FLASH_ZW __attribute__((section(".text.hot")))

#endif /* FLASH_ZW_H */
