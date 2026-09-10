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

/** USART1 baud for binary Euler stream (boot banner uses same UART). */
#ifndef PLATFORM_UART_BAUD
#define PLATFORM_UART_BAUD 921600u
#endif

/** CAN1 bitrate (bit/s). APB1 = 72 MHz with default clock tree. */
#ifndef PLATFORM_CAN_BITRATE
#define PLATFORM_CAN_BITRATE 1000000u
#endif

/** Standard 11-bit CAN ID for Euler frames. */
#ifndef PLATFORM_CAN_STD_ID
#define PLATFORM_CAN_STD_ID 0x321u
#endif

void platform_init(void);
void platform_delay_ms(uint32_t ms);
uint32_t platform_millis(void);

/** Blocking I2C master write: addr 7-bit, then reg + data[len]. */
int platform_i2c_write(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len);

/** Blocking I2C master write-then-read (repeated start). */
int platform_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);

void platform_uart_write(const char *s);
void platform_uart_printf(const char *fmt, ...);

/**
 * Polling TXE binary write (FLASH_ZW). No CR/LF translation — for 1 kHz packets.
 */
void platform_uart_write_bytes(const uint8_t *p, unsigned n);

/**
 * Pack and send 17-byte Euler UART packet (magic+seq+3×f32+xor). FLASH_ZW.
 * Returns bytes written (17) or 0 on null.
 */
unsigned platform_uart_send_euler_bin(uint16_t seq, float roll_deg, float pitch_deg, float yaw_deg);

/**
 * Non-blocking CAN TX of one 8-byte Euler frame (mailbox 0). FLASH_ZW.
 * Returns 0 on queued, -1 if mailbox busy (dropped after brief poll).
 */
int platform_can_send_euler(uint16_t seq, float roll_deg, float pitch_deg, float yaw_deg);

/** Cumulative CAN drops when mailbox was busy (debug). */
extern volatile uint32_t platform_can_drop_count;

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_H */
