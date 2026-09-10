/**
 * CH32V203 platform: HSI+PLL clock, USART1 debug, I2C1 on PB6/PB7.
 */
#include "platform.h"
#include "ch32v203_regs.h"
#include "flash_nzw.h"

#include <stdio.h>
#include <string.h>

uint32_t SystemCoreClock = HSI_VALUE;

/* Weak SystemInit called from startup before main. */
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
     * Without this, execute/fetch from non-zero-wait CodeFlash may be unreliable.
     * See also FLASH_Access_Clock_Cfg; we leave enhance clock at reset default.
     */
    if ((FLASH_CTLR & FLASH_CTLR_LOCK) != 0u) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
    FLASH_CTLR |= FLASH_CTLR_ENHANCE_READ;
    FLASH_CTLR |= FLASH_CTLR_LOCK;

    /* Prefer HSI * 18 = 144 MHz (CH32 EXTEN HSIPRE path, common bare-metal demo). */
    RCC_CTLR |= RCC_HSION;
    while ((RCC_CTLR & RCC_HSIRDY) == 0u) {
    }

    EXTEN_CTR |= EXTEN_HSIPRE;

    RCC_CFGR0 &= ~0xF0u;                 /* HPRE = /1 */
    RCC_CFGR0 &= ~(0x7u << 8);           /* clear PPRE1 */
    RCC_CFGR0 |= RCC_PPRE1_DIV2;         /* APB1 = SYSCLK/2 = 72 MHz */
    RCC_CFGR0 &= ~(0xFu << 18);
    RCC_CFGR0 |= RCC_PLL_MUL18;          /* PLL = HSI * 18 */
    /* PLLSRC=0: HSI/2 as PLL input on classic STM32; on CH32 with HSIPRE,
       mtkos path uses MUL18 without PLLSRC for 144 MHz from HSI. */

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
    /* Qingke SysTick: read CNTH then CNTL carefully for tear */
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
    /* Free-running up-counter @ HCLK, no interrupt — poll for timebase. */
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

FLASH_NZW static void usart1_init(uint32_t baud)
{
    RCC_APB2PCENR |= RCC_IOPAEN | RCC_USART1EN | RCC_AFIOEN;

    /* PA9 = USART1_TX AF PP 50MHz; PA10 = RX input pull-up */
    gpio_set_mode_high(GPIOA_BASE, 9u, GPIO_MODE_AF_PP_50);
    gpio_set_mode_high(GPIOA_BASE, 10u, GPIO_MODE_IN_PU);
    GPIO_OUTDR(GPIOA_BASE) |= (1u << 10);

    /* BRR for 16x oversampling: PCLK2 = SYSCLK (APB2 div1 default) */
    uint32_t pclk = SystemCoreClock;
    USART1_BRR = (pclk + baud / 2u) / baud;
    USART1_CTLR1 = USART_TE | USART_RE | USART_UE;
}

static void usart1_putc(char c)
{
    if (c == '\n') {
        usart1_putc('\r');
    }
    while ((USART1_STATR & USART_TXE) == 0u) {
    }
    USART1_DATAR = (uint16_t)(uint8_t)c;
}

void platform_uart_write(const char *s)
{
    if (s == NULL) {
        return;
    }
    while (*s) {
        usart1_putc(*s++);
    }
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

static int i2c_wait_flag(volatile uint16_t *reg, uint16_t mask, int set, uint32_t timeout)
{
    while (timeout--) {
        uint16_t v = *reg;
        if (set) {
            if ((v & mask) != 0u) {
                return 0;
            }
        } else if ((v & mask) == 0u) {
            return 0;
        }
    }
    return -1;
}

FLASH_NZW static void i2c1_init(void)
{
    RCC_APB2PCENR |= RCC_IOPBEN | RCC_AFIOEN;
    RCC_APB1PCENR |= RCC_I2C1EN;

    /* Default map: I2C1 SCL=PB6, SDA=PB7 (clear remap bit). */
    AFIO_PCFR1 &= ~AFIO_I2C1_REMAP;

    gpio_set_mode_low(GPIOB_BASE, 6u, GPIO_MODE_AF_OD_50);
    gpio_set_mode_low(GPIOB_BASE, 7u, GPIO_MODE_AF_OD_50);

    /* Software reset I2C */
    I2C1_CTLR1 |= I2C_CTLR1_SWRST;
    I2C1_CTLR1 &= (uint16_t)~I2C_CTLR1_SWRST;

    /* FREQ = PCLK1 MHz; PCLK1 = 72 MHz */
    uint32_t pclk1 = SystemCoreClock / 2u;
    uint16_t freq_mhz = (uint16_t)(pclk1 / 1000000u);
    I2C1_CTLR2 = freq_mhz & 0x3Fu;

    /* 100 kHz standard mode: CCR = PCLK1 / (2 * 100k) */
    uint16_t ccr = (uint16_t)(pclk1 / (100000u * 2u));
    if (ccr < 4u) {
        ccr = 4u;
    }
    I2C1_CKCFGR = ccr;

    I2C1_CTLR1 |= I2C_CTLR1_PE;
}

static int i2c_start_addr(uint8_t addr7, int read)
{
    if (i2c_wait_flag(&I2C1_STAR2, I2C_STAR2_BUSY, 0, 100000u) != 0) {
        I2C1_CTLR1 |= I2C_CTLR1_STOP;
        return -1;
    }
    I2C1_CTLR1 |= I2C_CTLR1_START;
    if (i2c_wait_flag(&I2C1_STAR1, I2C_STAR1_SB, 1, 100000u) != 0) {
        return -2;
    }
    I2C1_DATAR = (uint16_t)((addr7 << 1) | (read ? 1u : 0u));
    if (i2c_wait_flag(&I2C1_STAR1, I2C_STAR1_ADDR, 1, 100000u) != 0) {
        I2C1_CTLR1 |= I2C_CTLR1_STOP;
        return -3;
    }
    (void)I2C1_STAR2; /* clear ADDR */
    return 0;
}

int platform_i2c_write(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (i2c_start_addr(addr7, 0) != 0) {
        return -1;
    }
    I2C1_DATAR = reg;
    if (i2c_wait_flag(&I2C1_STAR1, I2C_STAR1_TXE, 1, 100000u) != 0) {
        I2C1_CTLR1 |= I2C_CTLR1_STOP;
        return -2;
    }
    for (uint16_t i = 0; i < len; i++) {
        I2C1_DATAR = data[i];
        if (i2c_wait_flag(&I2C1_STAR1, I2C_STAR1_TXE, 1, 100000u) != 0) {
            I2C1_CTLR1 |= I2C_CTLR1_STOP;
            return -3;
        }
    }
    if (i2c_wait_flag(&I2C1_STAR1, I2C_STAR1_BTF, 1, 100000u) != 0) {
        I2C1_CTLR1 |= I2C_CTLR1_STOP;
        return -4;
    }
    I2C1_CTLR1 |= I2C_CTLR1_STOP;
    return 0;
}

int platform_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (len == 0u || data == NULL) {
        return -1;
    }
    if (i2c_start_addr(addr7, 0) != 0) {
        return -1;
    }
    I2C1_DATAR = reg;
    if (i2c_wait_flag(&I2C1_STAR1, I2C_STAR1_TXE, 1, 100000u) != 0) {
        I2C1_CTLR1 |= I2C_CTLR1_STOP;
        return -2;
    }

    I2C1_CTLR1 |= I2C_CTLR1_START;
    if (i2c_wait_flag(&I2C1_STAR1, I2C_STAR1_SB, 1, 100000u) != 0) {
        return -3;
    }
    I2C1_DATAR = (uint16_t)((addr7 << 1) | 1u);
    if (i2c_wait_flag(&I2C1_STAR1, I2C_STAR1_ADDR, 1, 100000u) != 0) {
        I2C1_CTLR1 |= I2C_CTLR1_STOP;
        return -4;
    }

    if (len == 1u) {
        I2C1_CTLR1 &= (uint16_t)~I2C_CTLR1_ACK;
        (void)I2C1_STAR2;
        I2C1_CTLR1 |= I2C_CTLR1_STOP;
        if (i2c_wait_flag(&I2C1_STAR1, I2C_STAR1_RXNE, 1, 100000u) != 0) {
            return -5;
        }
        data[0] = (uint8_t)I2C1_DATAR;
        I2C1_CTLR1 |= I2C_CTLR1_ACK;
        return 0;
    }

    I2C1_CTLR1 |= I2C_CTLR1_ACK;
    (void)I2C1_STAR2;
    for (uint16_t i = 0; i < len; i++) {
        if (i == (uint16_t)(len - 1u)) {
            I2C1_CTLR1 &= (uint16_t)~I2C_CTLR1_ACK;
            I2C1_CTLR1 |= I2C_CTLR1_STOP;
        }
        if (i2c_wait_flag(&I2C1_STAR1, I2C_STAR1_RXNE, 1, 100000u) != 0) {
            return -6;
        }
        data[i] = (uint8_t)I2C1_DATAR;
    }
    I2C1_CTLR1 |= I2C_CTLR1_ACK;
    return 0;
}

FLASH_NZW void platform_init(void)
{
    /* SystemInit already ran from reset; re-assert clock var. */
    if (SystemCoreClock < 1000000u) {
        SystemInit();
    }
    systick_init();
    usart1_init(115200u);
    i2c1_init();
}
