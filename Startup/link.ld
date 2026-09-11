/*
 * Linker script for CH32V203G6U6
 *
 * Datasheet: advertised Flash bytes = zero-wait R0WAIT only.
 * V203 (non-RB): total CodeFlash ≈ 224 KB; NZW = 224K - R0WAIT.
 * G6U6: R0WAIT = 32 KB → NZW ≈ 192 KB @ 0x00008000
 * (FLASH base 0x00000000 as used by Qingke / this project).
 *
 * Layout:
 *   FLASH (32K ZW): vectors/reset/SystemInit + VQF 1 kHz hot graph + soft-float
 *   FLASH_NZW (188K): cold code; Mahony/Comp stored here as LMA of .algo_ram
 *   ALGO_CFG (4K @ 0x37000): persistent algo select flag (NOLOAD)
 *   ALGO_RAM (4K @ 0x20000000): SRAM execute window for alternate algos
 *   RAM (6K @ 0x20001000): .data / .bss / stack (1024B)
 */
ENTRY(_start)

__stack_size = 1024;
PROVIDE(_stack_size = __stack_size);

MEMORY
{
  FLASH (rx)     : ORIGIN = 0x00000000, LENGTH = 32K
  FLASH_NZW (rx) : ORIGIN = 0x00008000, LENGTH = 188K
  ALGO_CFG (r)   : ORIGIN = 0x00037000, LENGTH = 4K
  ALGO_RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 4K
  RAM (xrw)      : ORIGIN = 0x20001000, LENGTH = 6K
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

    build/Middleware/vqf-c/vqf.o(.text.updateMag .text.updateMag.*
                                 .text.getQuat9D .text.getQuat9D.*
                                 .text.getDelta .text.getDelta.*
                                 .text.getBiasEstimate .text.getBiasEstimate.*
                                 .text.setBiasEstimate .text.setBiasEstimate.*
                                 .text.getRestDetected .text.getRestDetected.*
                                 .text.getMagDistDetected .text.getMagDistDetected.*
                                 .text.getRelativeRestDeviations .text.getRelativeRestDeviations.*
                                 .text.getMagRefNorm .text.getMagRefNorm.*
                                 .text.getMagRefDip .text.getMagRefDip.*
                                 .text.setMagRef .text.setMagRef.*
                                 .text.setMagDistRejectionEnabled .text.setMagDistRejectionEnabled.*
                                 .text.setMotionBiasEstEnabled .text.setMotionBiasEstEnabled.*
                                 .text.setRestBiasEstEnabled .text.setRestBiasEstEnabled.*
                                 .text.setTauAcc .text.setTauAcc.*
                                 .text.setTauMag .text.setTauMag.*
                                 .text.setRestDetectionThresholds .text.setRestDetectionThresholds.*
                                 .text.initVqf .text.initVqf.*
                                 .text.setup .text.setup.*
                                 .text.resetState .text.resetState.*
                                 .text.init_params .text.init_params.*)
    */vqf.o(.text.updateMag .text.updateMag.*
            .text.getQuat9D .text.getQuat9D.*
            .text.getDelta .text.getDelta.*
            .text.getBiasEstimate .text.getBiasEstimate.*
            .text.setBiasEstimate .text.setBiasEstimate.*
            .text.getRestDetected .text.getRestDetected.*
            .text.getMagDistDetected .text.getMagDistDetected.*
            .text.getRelativeRestDeviations .text.getRelativeRestDeviations.*
            .text.getMagRefNorm .text.getMagRefNorm.*
            .text.getMagRefDip .text.getMagRefDip.*
            .text.setMagRef .text.setMagRef.*
            .text.setMagDistRejectionEnabled .text.setMagDistRejectionEnabled.*
            .text.setMotionBiasEstEnabled .text.setMotionBiasEstEnabled.*
            .text.setRestBiasEstEnabled .text.setRestBiasEstEnabled.*
            .text.setTauAcc .text.setTauAcc.*
            .text.setTauMag .text.setTauMag.*
            .text.setRestDetectionThresholds .text.setRestDetectionThresholds.*
            .text.initVqf .text.initVqf.*
            .text.setup .text.setup.*
            .text.resetState .text.resetState.*
            .text.init_params .text.init_params.*)

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

  .text_zw :
  {
    . = ALIGN(4);
    *(.text.handle_reset)
    *(.text.vector_handler)
    *(.text.SystemInit)
    *(.text.SysTick_Handler)
    *(.text.hot)
    *(.text.hot.*)

    build/Middleware/vqf-c/vqf.o(.text .text.*)
    */vqf.o(.text .text.*)
    */*/vqf.o(.text .text.*)
    */*/*/vqf.o(.text .text.*)

    *(.text.lsm6dsv_read*)
    *(.text.platform_spi*)
    *(.text.platform_lsm_cs*)
    *(.text.spi_*)
    *(.text.platform_millis*)
    *(.text.platform_delay*)
    *(.text.platform_uart_write_bytes*)
    *(.text.platform_uart_send_euler*)
    *(.text.platform_can_send*)
    *(.text.millideg_i16*)
    *(.text.systick*)

    *libgcc.a:*addsf3*(.text*)
    *libgcc.a:*subsf3*(.text*)
    *libgcc.a:*mulsf3*(.text*)
    *libgcc.a:*divsf3*(.text*)
    *libgcc.a:*eqsf2*(.text*)
    *libgcc.a:*gesf2*(.text*)
    *libgcc.a:*lesf2*(.text*)
    *libgcc.a:*unordsf2*(.text*)
    *libgcc.a:*adddf3*(.text*)
    *libgcc.a:*subdf3*(.text*)
    *libgcc.a:*muldf3*(.text*)
    *libgcc.a:*divdf3*(.text*)
    *libgcc.a:*eqdf2*(.text*)
    *libgcc.a:*gedf2*(.text*)
    *libgcc.a:*ledf2*(.text*)
    *libgcc.a:*unorddf2*(.text*)
    *libgcc.a:*extendsfdf2*(.text*)
    *libgcc.a:*truncdfsf2*(.text*)
    *libgcc.a:*save-restore*(.text*)
    *libgcc.a:*fixsfsi*(.text*)
    *libgcc.a:*floatsisf*(.text*)
    *libgcc.a:*fixunssfsi*(.text*)
    *libgcc.a:*floatunsisf*(.text*)

    *libc.a:*libm_machine_riscv_s_sqrt*(.text*)
    *libc.a:*libm_machine_riscv_sf_sqrt*(.text*)
    *libc.a:*libm_math_s_acos*(.text*)
    *libc.a:*libm_math_sf_asin*(.text*)
    *libc.a:*libm_math_sf_atan2*(.text*)
    *libc.a:*libm_math_sf_atan*(.text*)
    *libc.a:*libm_math_s_atan*(.text*)
    *libc.a:*libm_math_sf_sin*(.text*)
    *libc.a:*libm_math_sf_cos*(.text*)
    *libc.a:*libm_math_k_sin*(.text*)
    *libc.a:*libm_math_k_cos*(.text*)
    *libc.a:*libm_math_kf_sin*(.text*)
    *libc.a:*libm_math_kf_cos*(.text*)
    *libc.a:*libm_math_sf_rem_pio2*(.text*)
    *libc.a:*libm_math_kf_rem_pio2*(.text*)
    *libc.a:*libm_math_s_rem_pio2*(.text*)
    *libc.a:*libm_math_k_rem_pio2*(.text*)
    *libc.a:*libm_common_math_err*(.text*)
    *libc.a:*libm_machine_riscv_s_isnan*(.text*)
    *libc.a:*libm_machine_riscv_sf_isnan*(.text*)
    *libc.a:*memset*(.text*)
    *libc.a:*memcpy*(.text*)

    *libm.a:*sqrt*(.text*)
    *libm.a:*acos*(.text*)
    *libm.a:*asin*(.text*)
    *libm.a:*atan2*(.text*)
    *libm.a:*atan*(.text*)
    *libm.a:*sin*(.text*)
    *libm.a:*cos*(.text*)
    *libm.a:*isnan*(.text*)
    *libm.a:*rem_pio2*(.text*)
    *libm.a:*k_sin*(.text*)
    *libm.a:*k_cos*(.text*)

    /* newlib-nano (riscv-none-elf --specs=nano.specs) — selective hot math */
    *libm_nano.a:*sqrt*(.text*)
    *libm_nano.a:*acos*(.text*)
    *libm_nano.a:*asin*(.text*)
    *libm_nano.a:*atan2*(.text*)
    *libm_nano.a:*atan*(.text*)
    *libm_nano.a:*sin*(.text*)
    *libm_nano.a:*cos*(.text*)
    *libm_nano.a:*fabs*(.text*)
    *libm_nano.a:*nan*(.text*)
    *libm_nano.a:*rem_pio2*(.text*)
    *libm_nano.a:*k_sin*(.text*)
    *libm_nano.a:*k_cos*(.text*)
    *libm_nano.a:*kf_sin*(.text*)
    *libm_nano.a:*kf_cos*(.text*)
    *libc_nano.a:*memset*(.text*)
    *libc_nano.a:*memcpy*(.text*)

    . = ALIGN(4);
  } >FLASH AT>FLASH

  /*
   * Catch-all NZW text/rodata — exclude mahony/complementary objects so their
   * .text.algo.* / .rodata.algo.* land in .algo_ram (VMA=ALGO_RAM, LMA=NZW).
   * EXCLUDE also keeps their default .rodata (vtables) out; those vtables are
   * pulled explicitly into .text below via named sections if needed.
   *
   * Vtables (fusion_algo_mahony / fusion_algo_comp) are normal .rodata in
   * those .o files — we WANT them in NZW XIP, not in ALGO_RAM. So only exclude
   * .text.algo* / .rodata.algo* patterns by claiming .algo_ram AFTER with
   * explicit section names, and exclude those section names from wildcards.
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
   * Alternate fusion algos @ end of NZW image (LMA), execute from ALGO_RAM (VMA).
   * Startup copies _salgo_ram_lma → [_salgo_ram, _ealgo_ram).
   * Both Mahony + complementary fit in 3K — always copy both.
   *
   * IMPORTANT: must come AFTER .text/.fini so FLASH_NZW VMA==LMA for XIP code.
   * Input sections .text.algo* must not have been claimed by .text above.
   * GNU ld sorts more-specific input section names: we rely on .text.* also
   * matching .text.algo.mahony — so we MUST claim .algo_ram BEFORE .text, BUT
   * that breaks VMA==LMA. Solution: use INSERT or rename sections to not match.
   *
   * Practical fix used here: section names .algo_text.mahony (no .text. prefix)
   * — see flash attributes in mahony.c / complementary.c.
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

  /* .data LMA = end of last Flash-resident LMA (.algo_ram load end).
   * Location counter '.' after .algo_ram is ALGO_RAM VMA — use LOADADDR+SIZE. */
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
