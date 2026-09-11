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
#define RCC_SPI1EN      (1u << 12) /* APB2 */
#define RCC_USART1EN    (1u << 14) /* APB2 */
#define RCC_USART2EN    (1u << 17) /* APB1 */
#define RCC_I2C1EN      (1u << 21) /* APB1 — Mag I2C on schematic; unused here */

/* ---- FLASH (AHB) — enhance read for NZW CodeFlash ---- */
/* CTLR bits 24/22/25 from WCH ch32v20x_flash.c FLASH_Enhance_Mode /
 * FLASH_Access_Clock_Cfg (not fully enumerated in public bit headers). */
#define FLASH_R_BASE      0x40022000u
#define FLASH_ACTLR       REG32(FLASH_R_BASE + 0x00)
#define FLASH_KEYR        REG32(FLASH_R_BASE + 0x04)
#define FLASH_CTLR        REG32(FLASH_R_BASE + 0x10)
#define FLASH_KEY1        0x45670123u
#define FLASH_KEY2        0xCDEF89ABu
#define FLASH_STATR       REG32(FLASH_R_BASE + 0x0Cu)
#define FLASH_ADDR        REG32(FLASH_R_BASE + 0x14u)
#define FLASH_CTLR_PG     (1u << 0)
#define FLASH_CTLR_PER    (1u << 1)
#define FLASH_CTLR_STRT   (1u << 6)
#define FLASH_CTLR_LOCK   (1u << 7)
#define FLASH_CTLR_ENHANCE_READ (1u << 24) /* FLASH_Enhance_Mode(ENABLE) */
#define FLASH_CTLR_ENHANCE_CLK  (1u << 25) /* FLASH_Access_Clock_Cfg */
#define FLASH_STATR_BSY   (1u << 0)
#define FLASH_STATR_WRPRTERR (1u << 4)
#define FLASH_STATR_EOP   (1u << 5)

/* ---- EXTEN (CH32 specific) ---- */
#define EXTEN_CTR       REG32(0x40023800u)
#define EXTEN_HSIPRE    (1u << 4)

/* ---- GPIO ---- */
#define GPIOA_BASE      0x40010800u
#define GPIOB_BASE      0x40010C00u
#define GPIO_CFGLR(b)   REG32((b) + 0x00)
#define GPIO_CFGHR(b)   REG32((b) + 0x04)
#define GPIO_INDR(b)    REG32((b) + 0x08)
#define GPIO_OUTDR(b)   REG32((b) + 0x0C)
#define GPIO_BSHR(b)    REG32((b) + 0x10)

/* CNF/MODE nibbles: AF push-pull 50MHz=0b1011, Out PP 50MHz=0b0011, AF OD 50MHz=0b1111 */
#define GPIO_MODE_OUT_PP_50 0x3u
#define GPIO_MODE_AF_PP_50  0xBu
#define GPIO_MODE_AF_OD_50  0xFu
#define GPIO_MODE_IN_FLOAT  0x4u
#define GPIO_MODE_IN_PU     0x8u

/* ---- AFIO ---- */
#define AFIO_BASE       0x40010000u
#define AFIO_PCFR1      REG32(AFIO_BASE + 0x04)
#define AFIO_I2C1_REMAP (1u << 1) /* 0: PB6/PB7, 1: PB8/PB9 */
#define AFIO_USART2_REMAP (1u << 3) /* 0: PA2/PA3, 1: PD5/PD6 */

/* ---- USART2 (APB1) — CH32 default TX=PA2 RX=PA3 ---- */
#define USART2_BASE     0x40004400u
#define USART2_STATR    REG32(USART2_BASE + 0x00)
#define USART2_DATAR    REG32(USART2_BASE + 0x04)
#define USART2_BRR      REG32(USART2_BASE + 0x08)
#define USART2_CTLR1    REG32(USART2_BASE + 0x0C)

/* Shared USART bit defs (USART1/2 compatible layout) */
#define USART_TC        (1u << 6)
#define USART_RXNE      (1u << 5)
#define USART_TXE       (1u << 7)
#define USART_UE        (1u << 13)
#define USART_TE        (1u << 3)
#define USART_RE        (1u << 2)

/* Alias used by hot-path UART TX (USART2 on this board map) */
#define PLATFORM_USART_STATR USART2_STATR
#define PLATFORM_USART_DATAR USART2_DATAR

/* ---- SPI1 (APB2) — PA5 SCK / PA6 MISO / PA7 MOSI; software CS on PA4 ---- */
#define SPI1_BASE       0x40013000u
#define SPI1_CTLR1      REG16(SPI1_BASE + 0x00)
#define SPI1_CTLR2      REG16(SPI1_BASE + 0x04)
#define SPI1_STATR      REG16(SPI1_BASE + 0x08)
#define SPI1_DATAR      REG16(SPI1_BASE + 0x0C)

#define SPI_CTLR1_CPHA     (1u << 0)
#define SPI_CTLR1_CPOL     (1u << 1)
#define SPI_CTLR1_MSTR     (1u << 2)
#define SPI_CTLR1_BR_DIV8  (0x2u << 3) /* PCLK2/8 — 144/8=18 MHz @ APB2=SYSCLK */
#define SPI_CTLR1_SPE      (1u << 6)
#define SPI_CTLR1_SSI      (1u << 8)
#define SPI_CTLR1_SSM      (1u << 9)

#define SPI_STATR_RXNE  (1u << 0)
#define SPI_STATR_TXE   (1u << 1)
#define SPI_STATR_BSY   (1u << 7)

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

/* ---- CAN1 (bxCAN-like, shares 512B SRAM with USBD — leave USBD off) ---- */
#define CAN1_BASE       0x40006400u
#define CAN1_CTLR       REG32(CAN1_BASE + 0x00)   /* MCR */
#define CAN1_STATR      REG32(CAN1_BASE + 0x04)   /* MSR */
#define CAN1_TSTATR     REG32(CAN1_BASE + 0x08)   /* TSR */
#define CAN1_BTIMR      REG32(CAN1_BASE + 0x1C)   /* BTR */
#define CAN1_TXMIR0     REG32(CAN1_BASE + 0x180)
#define CAN1_TXMDTR0    REG32(CAN1_BASE + 0x184)
#define CAN1_TXMDLR0    REG32(CAN1_BASE + 0x188)
#define CAN1_TXMDHR0    REG32(CAN1_BASE + 0x18C)
#define CAN1_FCTLR      REG32(CAN1_BASE + 0x200)  /* FMR */
#define CAN1_FMCFGR     REG32(CAN1_BASE + 0x204)  /* FM1R */
#define CAN1_FSCFGR     REG32(CAN1_BASE + 0x20C)  /* FS1R */
#define CAN1_FAFIFOR    REG32(CAN1_BASE + 0x214)  /* FFA1R */
#define CAN1_FWR        REG32(CAN1_BASE + 0x21C)  /* FA1R */
#define CAN1_F0R1       REG32(CAN1_BASE + 0x240)
#define CAN1_F0R2       REG32(CAN1_BASE + 0x244)

#define RCC_CAN1EN      (1u << 25) /* APB1 */

#define CAN_CTLR_INRQ   (1u << 0)
#define CAN_CTLR_SLEEP  (1u << 1)
#define CAN_CTLR_NART   (1u << 4)
#define CAN_CTLR_ABOM   (1u << 6)
#define CAN_STATR_INAK  (1u << 0)
#define CAN_STATR_SLAK  (1u << 1)
#define CAN_TSTATR_TME0 (1u << 26)
#define CAN_TXMIR_TXRQ  (1u << 0)
#define CAN_FCTLR_FINIT (1u << 0)

/*
 * AFIO PCFR1 CAN_REMAP[1:0] (bits 14:13):
 *   00 = Remap1: CAN_RX=PA11, CAN_TX=PA12
 *   10 = Remap2: CAN_RX=PB8,  CAN_TX=PB9
 *   11 = Remap3: CAN_RX=PD0,  CAN_TX=PD1
 * CH32V203 has NO AF mapping of CAN onto PA2/PA3 (schematic AT32 nets).
 */
#define AFIO_CAN_REMAP_MASK (3u << 13)
#define AFIO_CAN_REMAP1     (0u << 13) /* PA11/PA12 */
#define AFIO_CAN_REMAP2     (2u << 13) /* PB8/PB9 */
#define AFIO_CAN_REMAP3     (3u << 13) /* PD0/PD1 */

#endif /* CH32V203_REGS_H */
