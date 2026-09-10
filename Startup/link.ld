/*
 * Linker script for CH32V203G6U6
 * Flash: 32 KB @ 0x00000000
 * SRAM:  10 KB @ 0x20000000
 */
ENTRY(_start)

__stack_size = 1024;
PROVIDE(_stack_size = __stack_size);

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 32K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 10K
}

SECTIONS
{
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
  } >FLASH AT>FLASH

  .fini :
  {
    KEEP(*(.fini))
    . = ALIGN(4);
  } >FLASH AT>FLASH

  PROVIDE(_etext = .);
  PROVIDE(_data_lma = .);

  .data :
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
  } >RAM AT>FLASH

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
