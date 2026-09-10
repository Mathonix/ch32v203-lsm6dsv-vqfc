/**
 * Minimal CH32V203 register map used by Platform HAL.
 * Addresses match WCH CH32V20x Reference Manual / public headers.
 */
#ifndef CH32V203_REGS_H
#define CH32V203_REGS_H

#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define REG16(addr) (*(volatile uint16_t *)(uintptr_t)(addr))

/* ---- RCC ---- */
#define RCC_BASE        0x40021000u
#define RCC_CTLR        REG32(RCC_BASE + 0x00)
#define RCC_CFGR0       REG32(RCC_BASE + 0x04)
#define RCC_APB2PRSTR   REG32(RCC_BASE + 0x0C)
#define RCC_APB1PRSTR   REG32(RCC_BASE + 0x10)
#define RCC_AHBPCENR    REG32(RCC_BASE + 0x14)
#define RCC_APB2PCENR   REG32(RCC_BASE + 0x18)
#define RCC_APB1PCENR   REG32(RCC_BASE + 0x1C)

#define RCC_PLLRDY      (1u << 25)
#define RCC_PLLON       (1u << 24)
#define RCC_HSERDY      (1u << 17)
#define RCC_HSEON       (1u << 16)
#define RCC_HSIRDY      (1u << 1)
#define RCC_HSION       (1u << 0)

#define RCC_SW_PLL      0x2u
#define RCC_SWS_MASK    0xCu
#define RCC_PLLSRC      (1u << 16)
#define RCC_PLL_MUL18   (0xFu << 18)
#define RCC_HPRE_DIV1   0u
#define RCC_PPRE1_DIV2  (4u << 8) /* APB1 = SYSCLK/2 when SYSCLK>36MHz typical */

#define RCC_IOPAEN      (1u << 2)
#define RCC_IOPBEN      (1u << 3)
#define RCC_AFIOEN      (1u << 0)
#define RCC_USART1EN    (1u << 14)
#define RCC_I2C1EN      (1u << 21) /* APB1 */

/* ---- EXTEN (CH32 specific) ---- */
#define EXTEN_CTR       REG32(0x40023800u)
#define EXTEN_HSIPRE    (1u << 4)

/* ---- GPIO ---- */
#define GPIOA_BASE      0x40010800u
#define GPIOB_BASE      0x40010C00u
#define GPIO_CFGLR(b)   REG32((b) + 0x00)
#define GPIO_CFGHR(b)   REG32((b) + 0x04)
#define GPIO_OUTDR(b)   REG32((b) + 0x0C)
#define GPIO_BSHR(b)    REG32((b) + 0x10)

/* CNF/MODE nibbles: AF push-pull 50MHz=0b1011, AF OD 50MHz=0b1111 */
#define GPIO_MODE_AF_PP_50  0xBu
#define GPIO_MODE_AF_OD_50  0xFu
#define GPIO_MODE_IN_PU     0x8u

/* ---- AFIO ---- */
#define AFIO_BASE       0x40010000u
#define AFIO_PCFR1      REG32(AFIO_BASE + 0x04)
#define AFIO_I2C1_REMAP (1u << 1) /* 0: PB6/PB7, 1: PB8/PB9 */

/* ---- USART1 ---- */
#define USART1_BASE     0x40013800u
#define USART1_STATR    REG32(USART1_BASE + 0x00)
#define USART1_DATAR    REG32(USART1_BASE + 0x04)
#define USART1_BRR      REG32(USART1_BASE + 0x08)
#define USART1_CTLR1    REG32(USART1_BASE + 0x0C)
#define USART_TC        (1u << 6)
#define USART_TXE       (1u << 7)
#define USART_UE        (1u << 13)
#define USART_TE        (1u << 3)
#define USART_RE        (1u << 2)

/* ---- I2C1 ---- */
#define I2C1_BASE       0x40005400u
#define I2C1_CTLR1      REG16(I2C1_BASE + 0x00)
#define I2C1_CTLR2      REG16(I2C1_BASE + 0x04)
#define I2C1_OADDR1     REG16(I2C1_BASE + 0x08)
#define I2C1_DATAR      REG16(I2C1_BASE + 0x10)
#define I2C1_STAR1      REG16(I2C1_BASE + 0x14)
#define I2C1_STAR2      REG16(I2C1_BASE + 0x18)
#define I2C1_CKCFGR     REG16(I2C1_BASE + 0x1C)

#define I2C_CTLR1_PE    (1u << 0)
#define I2C_CTLR1_START (1u << 8)
#define I2C_CTLR1_STOP  (1u << 9)
#define I2C_CTLR1_ACK   (1u << 10)
#define I2C_CTLR1_SWRST (1u << 15)

#define I2C_STAR1_SB    (1u << 0)
#define I2C_STAR1_ADDR  (1u << 1)
#define I2C_STAR1_BTF   (1u << 2)
#define I2C_STAR1_STOPF (1u << 4)
#define I2C_STAR1_RXNE  (1u << 6)
#define I2C_STAR1_TXE   (1u << 7)
#define I2C_STAR1_AF    (1u << 10)
#define I2C_STAR2_BUSY  (1u << 1)

/* ---- SysTick (Qingke) ---- */
#define STK_BASE        0xE000F000u
#define STK_CTLR        REG32(STK_BASE + 0x00)
#define STK_SR          REG32(STK_BASE + 0x04)
#define STK_CNTL        REG32(STK_BASE + 0x08)
#define STK_CNTH        REG32(STK_BASE + 0x0C)
#define STK_CMPLR       REG32(STK_BASE + 0x10)
#define STK_CMPHR       REG32(STK_BASE + 0x14)
#define STK_STE         (1u << 0)
#define STK_STIE        (1u << 1)
#define STK_STCLK       (1u << 2) /* 1: HCLK, 0: HCLK/8 */
#define STK_STRE        (1u << 3)

#ifndef HSI_VALUE
#define HSI_VALUE       8000000u
#endif

#endif /* CH32V203_REGS_H */
