/**
 * @file platform.h
 * @brief Thin board abstraction for CH32V203 (+ host stubs possible later).
 *
 * Schematic board MCU is AT32F423KCU7-4; this firmware targets CH32V203 with
 * matching net names where the AF coincides (SPI1 PA4–PA7). See README pin table
 * for CH32 UART/CAN AF vs schematic nets.
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

/** USART2 (PA2/PA3) baud for binary Euler stream (boot banner uses same UART). */
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

/**
 * SPI1 full-duplex xfer with soft CS (PA4). FLASH_ZW.
 * @param tx  bytes to send, or NULL to clock out 0xFF
 * @param rx  receive buffer, or NULL to discard
 * @param len byte count
 * @return 0 on success, negative on timeout
 */
int platform_spi_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

/** Assert (1) or deassert (0) LSM_CS on PA4 (active low when asserted). FLASH_ZW. */
void platform_lsm_cs(int assert_low);

void platform_uart_write(const char *s);
void platform_uart_printf(const char *fmt, ...);

/** Non-blocking UART RX; returns 0..255 or -1 if empty. */
int platform_uart_getc_nonblock(void);

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
