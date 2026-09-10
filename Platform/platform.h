/**
 * @file platform.h
 * @brief Thin board abstraction for CH32V203 (+ host stubs possible later).
 */
#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/** System core clock after platform_init (Hz). */
extern uint32_t SystemCoreClock;

void platform_init(void);
void platform_delay_ms(uint32_t ms);
uint32_t platform_millis(void);

/** Blocking I2C master write: addr 7-bit, then reg + data[len]. */
int platform_i2c_write(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len);

/** Blocking I2C master write-then-read (repeated start). */
int platform_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);

void platform_uart_write(const char *s);
void platform_uart_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_H */
