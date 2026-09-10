/*
 * Linker script for CH32V203G6U6
 *
 * Datasheet: advertised Flash bytes = zero-wait R0WAIT only.
 * V203 (non-RB): total CodeFlash ≈ 224 KB; NZW = 224K - R0WAIT.
 * G6U6: R0WAIT = 32 KB → NZW ≈ 192 KB @ 0x00008000
 * (FLASH base 0x00000000 as used by Qingke / this project).
 *
 * Layout (section name .text_nzw — not WCH .stext):
 *   FLASH (32K ZW):
 *     .init, .vector, handle_reset, IRQ stubs, SystemInit, hot path
 *   FLASH_NZW (192K):
 *     .text_nzw (cold/init + whole vqf.o), default .text/.rodata/.fini
 *
 * Note: this toolchain's ld (binutils 2.44) rejects MEMORY region-list
 * overflow (`>FLASH FLASH_NZW`); we place explicitly instead.
 */
ENTRY(_start)

__stack_size = 1024;
PROVIDE(_stack_size = __stack_size);

MEMORY
{
  /* Zero-wait R0WAIT */
  FLASH (rx)     : ORIGIN = 0x00000000, LENGTH = 32K
  /* Non-zero-wait remainder of 224 KB CodeFlash */
  FLASH_NZW (rx) : ORIGIN = 0x00008000, LENGTH = 192K
  RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 10K
}

SECTIONS
{
  /* ---- Zero-wait: reset trampoline ---- */
  .init :
  {
    _sinit = .;
    . = ALIGN(4);
    KEEP(*(SORT_NONE(.init)))
    . = ALIGN(4);
    _einit = .;
  } >FLASH AT>FLASH

  /* ---- Zero-wait: vector table ---- */
  .vector :
  {
    *(.vector)
    . = ALIGN(64);
  } >FLASH AT>FLASH

  /*
   * Zero-wait: critical reset / IRQ / early SystemInit + hot sample path.
   * Keep this under 32K; cold/VQF live in FLASH_NZW below.
   */
  .text_zw :
  {
    . = ALIGN(4);
    *(.text.handle_reset)
    *(.text.vector_handler)
    *(.text.SystemInit)
    *(.text.hot)
    *(.text.hot.*)
    /* High-rate VQF / IMU / I2C / timebase */
    *(.text.updateGyr*)
    *(.text.updateAcc*)
    *(.text.getQuat6D*)
    *(.text.lsm6dsv_read*)
    *(.text.platform_i2c*)
    *(.text.platform_millis*)
    *(.text.platform_delay*)
    *(.text.systick*)
    *(.text.platform_uart_write*)
    . = ALIGN(4);
  } >FLASH AT>FLASH

  /*
   * Non-zero-wait: cold init, VQF bulk, libc/libm, printf, default text.
   */
  .text_nzw :
  {
    . = ALIGN(4);
    *(.text_nzw)
    *(.text_nzw.*)
    *(.rodata_nzw)
    *(.rodata_nzw.*)
    /* Remaining VQF sections (hot updateGyr/Acc/getQuat6D already in .text_zw).
     * ld wildcards do not cross '/', so match nested build path explicitly. */
    build/Middleware/vqf-c/vqf.o(.text .text.* .rodata .rodata.* .srodata*)
    */vqf.o(.text .text.* .rodata .rodata.* .srodata*)
    */*/vqf.o(.text .text.* .rodata .rodata.* .srodata*)
    */*/*/vqf.o(.text .text.* .rodata .rodata.* .srodata*)
    . = ALIGN(4);
  } >FLASH_NZW AT>FLASH_NZW

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

  /* Contiguous image: VMA end of last flash section == LMA for .data */
  PROVIDE(_etext = .);
  PROVIDE(_data_lma = .);

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

  .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size :
  {
    . = ALIGN(4);
    PROVIDE(_susrstack = .);
    . = . + __stack_size;
    PROVIDE(_eusrstack = .);
  } >RAM
}
