/*
 * Linker script for CH32V203G6U6
 *
 * Datasheet: advertised Flash bytes = zero-wait R0WAIT only.
 * V203 (non-RB): total CodeFlash ≈ 224 KB; NZW = 224K - R0WAIT.
 * G6U6: R0WAIT = 32 KB → NZW ≈ 192 KB @ 0x00008000
 * (FLASH base 0x00000000 as used by Qingke / this project).
 *
 * Layout (Option B — HOT business in SRAM; __divdi3 family stays ZW):
 *   FLASH (32K ZW): vectors/reset/SystemInit/SysTick + int64 div helpers
 *   FLASH_NZW (188K): cold code; Mahony/Comp + HOT_RAM stored as LMA
 *   ALGO_CFG (4K @ 0x37000): persistent algo select flag (NOLOAD)
 *   HOT_RAM (4K @ 0x20000000): 1 kHz VQF-fxp business hot path (execute)
 *   ALGO_RAM (4K @ 0x20001000): Mahony + complementary (execute)
 *   RAM (2K @ 0x20002000): .data / .bss / stack (1024B)
 *
 * Total SRAM = 10 KB. HOT and ALGO are non-overlapping so SPI helpers in
 * HOT_RAM remain valid when Mahony/Comp run from ALGO_RAM.
 */
ENTRY(_start)

__stack_size = 1024;
PROVIDE(_stack_size = __stack_size);

MEMORY
{
  FLASH (rx)     : ORIGIN = 0x00000000, LENGTH = 32K
  FLASH_NZW (rx) : ORIGIN = 0x00008000, LENGTH = 188K
  ALGO_CFG (r)   : ORIGIN = 0x00037000, LENGTH = 4K
  HOT_RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 4K
  ALGO_RAM (xrw) : ORIGIN = 0x20001000, LENGTH = 4K
  RAM (xrw)      : ORIGIN = 0x20002000, LENGTH = 2K
}

SECTIONS
{
  .text_nzw :
  {
    . = ALIGN(4);
    *(.text_nzw)
    *(.text_nzw.*)
    *(.rodata_nzw)
    *(.rodata_nzw.*)


    . = ALIGN(4);
  } >FLASH_NZW AT>FLASH_NZW

  .init :
  {
    _sinit = .;
    . = ALIGN(4);
    KEEP(*(SORT_NONE(.init)))
    . = ALIGN(4);
    _einit = .;
  } >FLASH AT>FLASH

  .vector :
  {
    *(.vector)
    . = ALIGN(64);
  } >FLASH AT>FLASH

  /*
   * Zero-wait resident: boot/reset, SysTick stub, libc helpers, int64 div.
   * 1 kHz business hot path lives in .text.hot_ram (SRAM), not here.
   */
  .text_zw :
  {
    . = ALIGN(4);
    *(.text.handle_reset)
    *(.text.vector_handler)
    *(.text.SystemInit)
    *(.text.SysTick_Handler)

    *libc.a:*memset*(.text*)
    *libc.a:*memcpy*(.text*)
    *libc_nano.a:*memset*(.text*)
    *libc_nano.a:*memcpy*(.text*)
    *libgcc.a:*save-restore*(.text*)
    /* 64-bit integer helpers stay in ZW (Option B) */
    *libgcc.a:*_divdi3.o(.text .text.*)
    *libgcc.a:*_udivdi3.o(.text .text.*)
    *libgcc.a:*_moddi3.o(.text .text.*)
    *libgcc.a:*_umoddi3.o(.text .text.*)
    *libgcc.a:*muldi3.o(.text .text.*)
    *libgcc.a:*divdi3*(.text*)
    *libgcc.a:*udivdi3*(.text*)


    . = ALIGN(4);
  } >FLASH AT>FLASH

  /*
   * 1 kHz fixed-point hot path: VMA = HOT_RAM (SRAM execute),
   * LMA = FLASH (ZW) for boot memcpy. FLASH_ZW → .text.hot_ram.
   */
  .text.hot_ram : ALIGN(4)
  {
    PROVIDE(_shot_ram = .);
    *(.text.hot_ram)
    *(.text.hot_ram.*)
    /* Legacy name still accepted */
    *(.text.hot)
    *(.text.hot.*)

    build/Middleware/vqf-fxp/vqf_fxp.o(.text .text.*)
    */vqf_fxp.o(.text .text.*)
    */*/vqf_fxp.o(.text .text.*)
    */*/*/vqf_fxp.o(.text .text.*)

    *(.text.lsm6dsv_read*)
    *(.text.vqfx_*)
    *(.text.platform_uart_send_euler_i16*)
    *(.text.platform_can_send_euler_i16*)
    *(.text.mdeg_to_i16*)
    *(.text.fusion_run_1khz*)
    *(.text.platform_spi*)
    *(.text.platform_lsm_cs*)
    *(.text.spi_*)
    *(.text.platform_millis*)
    *(.text.systick_count64*)
    *(.text.platform_uart_write_bytes*)
    *(.text.millideg_i16*)

    . = ALIGN(4);
    PROVIDE(_ehot_ram = .);
  } >HOT_RAM AT>FLASH

  PROVIDE(_shot_ram_lma = LOADADDR(.text.hot_ram));

  /*
   * Catch-all NZW text/rodata. Alternate algos use .algo_text.* (no .text.
   * prefix) so they are not claimed here — see .algo_ram below.
   */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text.*)
    *(.rodata)
    *(.rodata*)
    *(.srodata*)
    *(.gnu.linkonce.t.*)
    . = ALIGN(4);
  } >FLASH_NZW AT>FLASH_NZW

  .fini :
  {
    KEEP(*(.fini))
    . = ALIGN(4);
  } >FLASH_NZW AT>FLASH_NZW

  /*
   * Alternate fusion algos: VMA=ALGO_RAM, LMA=FLASH_NZW.
   * Startup copies _salgo_ram_lma → [_salgo_ram, _ealgo_ram).
   * Must come AFTER .text/.fini so FLASH_NZW VMA==LMA for XIP code.
   */
  .algo_ram : ALIGN(4)
  {
    PROVIDE(_salgo_ram = .);
    *(.algo_text.mahony)
    *(.algo_text.mahony.*)
    *(.algo_rodata.mahony)
    *(.algo_rodata.mahony.*)
    *(.algo_text.comp)
    *(.algo_text.comp.*)
    *(.algo_rodata.comp)
    *(.algo_rodata.comp.*)
    *(.algo_text)
    *(.algo_text.*)
    *(.algo_rodata)
    *(.algo_rodata.*)
    . = ALIGN(4);
    PROVIDE(_ealgo_ram = .);
  } >ALGO_RAM AT>FLASH_NZW

  PROVIDE(_salgo_ram_lma = LOADADDR(.algo_ram));

  /* .data LMA follows last Flash-resident NZW LMA (.algo_ram).
   * HOT_RAM LMA lives in FLASH (ZW) separately — not contiguous. */
  PROVIDE(_etext = LOADADDR(.algo_ram) + SIZEOF(.algo_ram));
  PROVIDE(_data_lma = _etext);

  .data : AT(_etext)
  {
    . = ALIGN(4);
    PROVIDE(_data_vma = .);
    *(.gnu.linkonce.r.*)
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    . = ALIGN(8);
    PROVIDE(__global_pointer$ = . + 0x800);
    *(.sdata .sdata.* .sdata*)
    *(.gnu.linkonce.s.*)
    . = ALIGN(8);
    *(.sdata2 .sdata2.*)
    *(.gnu.linkonce.s2.*)
    . = ALIGN(4);
    PROVIDE(_edata = .);
  } >RAM

  .bss :
  {
    . = ALIGN(4);
    PROVIDE(_sbss = .);
    *(.sbss*)
    *(.gnu.linkonce.sb.*)
    *(.bss*)
    *(.gnu.linkonce.b.*)
    *(COMMON*)
    . = ALIGN(4);
    PROVIDE(_ebss = .);
  } >RAM AT>RAM

  PROVIDE(_end = _ebss);
  PROVIDE(end = .);

  .algo_cfg (NOLOAD) :
  {
    KEEP(*(.algo_cfg))
    . = ALIGN(4);
  } >ALGO_CFG

  .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size :
  {
    . = ALIGN(4);
    PROVIDE(_susrstack = .);
    . = . + __stack_size;
    PROVIDE(_eusrstack = .);
  } >RAM
}
