/**
 * CH32V203 platform: HSI+PLL clock, USART2, SPI1 for LSM6DSV, CAN1.
 *
 * Schematic board MCU is AT32F423KCU7-4; this FW targets CH32V203.
 *   LSM SPI PA4–PA7: exact net/AF match (SPI1 + soft CS).
 *   UART schematic nets PA0/PA1: CH32 RM maps only USART2 CTS/RTS there —
 *     no USART TX/RX AF. Firmware uses USART2 data pins PA2=TX / PA3=RX
 *     (CH32 default, no remap). Keep 921600 binary + boot printf.
 *   CAN schematic nets PA2/PA3: CH32 CAN remaps are PA11/12, PB8/9, PD0/1
 *     only — no PA2/PA3. Firmware uses Remap1 PA11/PA12 (USBD off).
 *     Note: USART2 also claims PA2/PA3 on CH32, so schematic CAN nets and
 *     CH32 USART2 pins coincide; choose USART2 for UART, CAN on Remap1.
 *   LSM_INT1/INT2 PB0/PB1: inputs, unused. Mag I2C PB6/PB7: out of scope.
 */
#include "platform.h"
#include "ch32v203_regs.h"
#include "flash_nzw.h"
#include "flash_zw.h"

#include <stdio.h>
#include <string.h>

uint32_t SystemCoreClock = HSI_VALUE;
volatile uint32_t platform_can_drop_count = 0u;

void SystemInit(void);

static void gpio_set_mode_low(uint32_t gpiobase, unsigned pin, uint32_t mode_nibble)
{
    uint32_t shift = pin * 4u;
    uint32_t cfg = GPIO_CFGLR(gpiobase);
    cfg &= ~(0xFu << shift);
    cfg |= (mode_nibble & 0xFu) << shift;
    GPIO_CFGLR(gpiobase) = cfg;
}

static void gpio_set_mode_high(uint32_t gpiobase, unsigned pin, uint32_t mode_nibble)
{
    uint32_t shift = (pin - 8u) * 4u;
    uint32_t cfg = GPIO_CFGHR(gpiobase);
    cfg &= ~(0xFu << shift);
    cfg |= (mode_nibble & 0xFu) << shift;
    GPIO_CFGHR(gpiobase) = cfg;
}

void SystemInit(void)
{
    /*
     * Enable FLASH enhance read mode BEFORE any code in FLASH_NZW runs.
     * Source: WCH EVT ch32v20x_flash.c FLASH_Enhance_Mode(ENABLE) sets
     * FLASH->CTLR bit 24. Unlock first (same keys as programming unlock).
     */
    if ((FLASH_CTLR & FLASH_CTLR_LOCK) != 0u) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
    FLASH_CTLR |= FLASH_CTLR_ENHANCE_READ;
    FLASH_CTLR |= FLASH_CTLR_LOCK;

    /* Prefer HSI * 18 = 144 MHz (CH32 EXTEN HSIPRE path). */
    RCC_CTLR |= RCC_HSION;
    while ((RCC_CTLR & RCC_HSIRDY) == 0u) {
    }

    EXTEN_CTR |= EXTEN_HSIPRE;

    RCC_CFGR0 &= ~0xF0u;                 /* HPRE = /1 */
    RCC_CFGR0 &= ~(0x7u << 8);           /* clear PPRE1 */
    RCC_CFGR0 |= RCC_PPRE1_DIV2;         /* APB1 = SYSCLK/2 = 72 MHz */
    RCC_CFGR0 &= ~(0xFu << 18);
    RCC_CFGR0 |= RCC_PLL_MUL18;          /* PLL = HSI * 18 */

    RCC_CTLR |= RCC_PLLON;
    while ((RCC_CTLR & RCC_PLLRDY) == 0u) {
    }

    RCC_CFGR0 = (RCC_CFGR0 & ~0x3u) | RCC_SW_PLL;
    while ((RCC_CFGR0 & RCC_SWS_MASK) != (RCC_SW_PLL << 2)) {
    }

    SystemCoreClock = 144000000u;
}

static uint64_t systick_count64(void)
{
    uint32_t hi1, lo, hi2;
    do {
        hi1 = STK_CNTH;
        lo = STK_CNTL;
        hi2 = STK_CNTH;
    } while (hi1 != hi2);
    return ((uint64_t)hi1 << 32) | lo;
}

static void systick_init(void)
{
    STK_CTLR = 0u;
    STK_CNTL = 0u;
    STK_CNTH = 0u;
    STK_CMPLR = 0xFFFFFFFFu;
    STK_CMPHR = 0xFFFFFFFFu;
    STK_SR = 0u;
    STK_CTLR = STK_STE | STK_STCLK | STK_STRE;
}

uint32_t platform_millis(void)
{
    return (uint32_t)(systick_count64() / (SystemCoreClock / 1000u));
}

void platform_delay_ms(uint32_t ms)
{
    uint64_t start = systick_count64();
    uint64_t ticks = (uint64_t)ms * (SystemCoreClock / 1000u);
    while ((systick_count64() - start) < ticks) {
    }
}

/* ---- USART2 @ PA2 TX / PA3 RX (CH32 RM; schematic UART nets are PA0/PA1 on AT32) ---- */

FLASH_NZW static void usart2_init(uint32_t baud)
{
    RCC_APB2PCENR |= RCC_IOPAEN | RCC_AFIOEN;
    RCC_APB1PCENR |= RCC_USART2EN;

    /* No USART2 remap → PA2=TX, PA3=RX (PA0/PA1 are CTS/RTS only on CH32) */
    AFIO_PCFR1 &= ~AFIO_USART2_REMAP;

    gpio_set_mode_low(GPIOA_BASE, 2u, GPIO_MODE_AF_PP_50);
    gpio_set_mode_low(GPIOA_BASE, 3u, GPIO_MODE_IN_PU);
    GPIO_OUTDR(GPIOA_BASE) |= (1u << 3);

    /* BRR: PCLK1 = SYSCLK/2 = 72 MHz */
    uint32_t pclk1 = SystemCoreClock / 2u;
    USART2_BRR = (pclk1 + baud / 2u) / baud;
    USART2_CTLR1 = USART_TE | USART_RE | USART_UE;
}

static void usart2_putc(char c)
{
    if (c == '\n') {
        usart2_putc('\r');
    }
    while ((USART2_STATR & USART_TXE) == 0u) {
    }
    USART2_DATAR = (uint16_t)(uint8_t)c;
}

void platform_uart_write(const char *s)
{
    if (s == NULL) {
        return;
    }
    while (*s) {
        usart2_putc(*s++);
    }
}

FLASH_NZW int platform_uart_getc_nonblock(void)
{
    if ((USART2_STATR & USART_RXNE) == 0u) {
        return -1;
    }
    return (int)(uint8_t)USART2_DATAR;
}

FLASH_NZW void platform_uart_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    platform_uart_write(buf);
}

/* ---- SPI1 + soft CS (LSM6DSV) — mode 3, ZW hot path ---- */

FLASH_ZW void platform_lsm_cs(int assert_low)
{
    if (assert_low) {
        GPIO_BSHR(GPIOA_BASE) = (1u << (4u + 16u)); /* BR4: clear PA4 */
    } else {
        GPIO_BSHR(GPIOA_BASE) = (1u << 4u); /* BS4: set PA4 */
    }
}

FLASH_ZW int platform_spi_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t out = tx ? tx[i] : 0xFFu;
        uint32_t guard = 100000u;
        while ((SPI1_STATR & SPI_STATR_TXE) == 0u) {
            if (--guard == 0u) {
                return -1;
            }
        }
        SPI1_DATAR = out;
        guard = 100000u;
        while ((SPI1_STATR & SPI_STATR_RXNE) == 0u) {
            if (--guard == 0u) {
                return -2;
            }
        }
        uint8_t in = (uint8_t)SPI1_DATAR;
        if (rx) {
            rx[i] = in;
        }
    }
    return 0;
}

FLASH_NZW static void spi1_init(void)
{
    RCC_APB2PCENR |= RCC_IOPAEN | RCC_IOPBEN | RCC_SPI1EN | RCC_AFIOEN;

    /* PA4 = LSM_CS: GPIO PP, idle high (active low) */
    gpio_set_mode_low(GPIOA_BASE, 4u, GPIO_MODE_OUT_PP_50);
    platform_lsm_cs(0);

    /* PA5=SCK, PA6=MISO, PA7=MOSI — SPI1 default AF (no SPI1 remap) */
    gpio_set_mode_low(GPIOA_BASE, 5u, GPIO_MODE_AF_PP_50);
    gpio_set_mode_low(GPIOA_BASE, 6u, GPIO_MODE_IN_PU); /* MISO input w/ pull-up */
    GPIO_OUTDR(GPIOA_BASE) |= (1u << 6);
    gpio_set_mode_low(GPIOA_BASE, 7u, GPIO_MODE_AF_PP_50);

    /* PB0=LSM_INT1, PB1=LSM_INT2 — floating inputs, unused */
    gpio_set_mode_low(GPIOB_BASE, 0u, GPIO_MODE_IN_FLOAT);
    gpio_set_mode_low(GPIOB_BASE, 1u, GPIO_MODE_IN_FLOAT);

    /*
     * SPI mode 3 (CPOL=1 CPHA=1): idle clock high, sample on trailing edge.
     * LSM6DSV datasheet: compatible with SPI modes 0 and 3.
     * Master, SSM/SSI soft NSS, BR = /8 → 18 MHz @ 144 MHz PCLK2.
     */
    SPI1_CTLR1 = 0u;
    SPI1_CTLR1 = (uint16_t)(SPI_CTLR1_MSTR | SPI_CTLR1_SSM | SPI_CTLR1_SSI |
                            SPI_CTLR1_CPOL | SPI_CTLR1_CPHA | SPI_CTLR1_BR_DIV8);
    SPI1_CTLR1 |= SPI_CTLR1_SPE;
}

/* ---- UART binary TX (ZW hot path) ---- */

FLASH_ZW void platform_uart_write_bytes(const uint8_t *p, unsigned n)
{
    if (p == NULL) {
        return;
    }
    for (unsigned i = 0; i < n; i++) {
        while ((USART2_STATR & USART_TXE) == 0u) {
        }
        USART2_DATAR = (uint16_t)p[i];
    }
}

FLASH_ZW unsigned platform_uart_send_euler_bin(uint16_t seq, float roll_deg,
                                               float pitch_deg, float yaw_deg)
{
    uint8_t pkt[17];
    pkt[0] = 0xA5u;
    pkt[1] = 0x5Au;
    pkt[2] = (uint8_t)(seq & 0xFFu);
    pkt[3] = (uint8_t)((seq >> 8) & 0xFFu);
    union {
        float f;
        uint8_t b[4];
    } u;
    u.f = roll_deg;
    pkt[4] = u.b[0];
    pkt[5] = u.b[1];
    pkt[6] = u.b[2];
    pkt[7] = u.b[3];
    u.f = pitch_deg;
    pkt[8] = u.b[0];
    pkt[9] = u.b[1];
    pkt[10] = u.b[2];
    pkt[11] = u.b[3];
    u.f = yaw_deg;
    pkt[12] = u.b[0];
    pkt[13] = u.b[1];
    pkt[14] = u.b[2];
    pkt[15] = u.b[3];
    uint8_t x = 0u;
    for (unsigned i = 0; i < 16u; i++) {
        x ^= pkt[i];
    }
    pkt[16] = x;
    platform_uart_write_bytes(pkt, 17u);
    return 17u;
}

/* ---- CAN1 init (NZW) + TX (ZW) ---- */

FLASH_NZW static void can1_init(void)
{
    /*
     * Schematic CAN_RX/TX = PA2/PA3 (AT32 CAN2 MUX9).
     * CH32V203 CAN1 remaps (AFIO_PCFR1 CAN_REMAP[1:0]):
     *   Remap1 00 → PA11 RX / PA12 TX
     *   Remap2 10 → PB8 RX / PB9 TX
     *   Remap3 11 → PD0 RX / PD1 TX
     * No PA2/PA3 CAN AF. Use Remap1; USBD left off (shares 512B SRAM).
     *
     * Bitrate: APB1 = 72 MHz → BRP=6, TS1=8, TS2=3 → 1 Mbit, sample ≈ 75%.
     */
    RCC_APB2PCENR |= RCC_IOPAEN | RCC_AFIOEN;
    RCC_APB1PCENR |= RCC_CAN1EN;

    AFIO_PCFR1 = (AFIO_PCFR1 & ~AFIO_CAN_REMAP_MASK) | AFIO_CAN_REMAP1;

    gpio_set_mode_high(GPIOA_BASE, 11u, GPIO_MODE_IN_PU);
    GPIO_OUTDR(GPIOA_BASE) |= (1u << 11);
    gpio_set_mode_high(GPIOA_BASE, 12u, GPIO_MODE_AF_PP_50);

    CAN1_CTLR &= ~CAN_CTLR_SLEEP;
    {
        uint32_t t = 100000u;
        while ((CAN1_STATR & CAN_STATR_SLAK) != 0u && t--) {
        }
    }
    CAN1_CTLR |= CAN_CTLR_INRQ;
    {
        uint32_t t = 100000u;
        while ((CAN1_STATR & CAN_STATR_INAK) == 0u && t--) {
        }
    }

    CAN1_CTLR |= CAN_CTLR_ABOM | CAN_CTLR_NART;

#if PLATFORM_CAN_BITRATE == 1000000u
    CAN1_BTIMR = (0u << 24) | (2u << 20) | (7u << 16) | 5u;
#else
#error "Only PLATFORM_CAN_BITRATE 1000000 supported in this build; adjust BTIMR"
#endif

    CAN1_FCTLR |= CAN_FCTLR_FINIT;
    CAN1_FMCFGR &= ~1u;
    CAN1_FSCFGR |= 1u;
    CAN1_FAFIFOR &= ~1u;
    CAN1_F0R1 = 0u;
    CAN1_F0R2 = 0u;
    CAN1_FWR |= 1u;
    CAN1_FCTLR &= ~CAN_FCTLR_FINIT;

    CAN1_CTLR &= ~CAN_CTLR_INRQ;
    {
        uint32_t t = 100000u;
        while ((CAN1_STATR & CAN_STATR_INAK) != 0u && t--) {
        }
    }
}

FLASH_ZW static int16_t millideg_i16(float deg)
{
    float md = deg * 1000.0f;
    if (md > 32767.0f) {
        md = 32767.0f;
    } else if (md < -32768.0f) {
        md = -32768.0f;
    }
    return (int16_t)md;
}

FLASH_ZW int platform_can_send_euler(uint16_t seq, float roll_deg, float pitch_deg,
                                     float yaw_deg)
{
    unsigned spins = 2u;
    while ((CAN1_TSTATR & CAN_TSTATR_TME0) == 0u) {
        if (spins == 0u) {
            platform_can_drop_count++;
            return -1;
        }
        spins--;
    }

    int16_t r = millideg_i16(roll_deg);
    int16_t p = millideg_i16(pitch_deg);
    int16_t y = millideg_i16(yaw_deg);

    uint32_t id = ((uint32_t)(PLATFORM_CAN_STD_ID & 0x7FFu) << 21);
    CAN1_TXMDTR0 = 8u;
    CAN1_TXMDLR0 = ((uint32_t)(uint16_t)r) |
                   ((uint32_t)(uint16_t)p << 16);
    CAN1_TXMDHR0 = ((uint32_t)(uint16_t)y) |
                   ((uint32_t)seq << 16);
    CAN1_TXMIR0 = id | CAN_TXMIR_TXRQ;
    return 0;
}

FLASH_NZW void platform_init(void)
{
    if (SystemCoreClock < 1000000u) {
        SystemInit();
    }
    systick_init();
    usart2_init(PLATFORM_UART_BAUD);
    spi1_init();
    can1_init();
}
