/*
 * Linker script for CH32V203G6U6
 *
 * Datasheet: advertised Flash bytes = zero-wait R0WAIT only.
 * V203 (non-RB): total CodeFlash ≈ 224 KB; NZW = 224K - R0WAIT.
 * G6U6: R0WAIT = 32 KB → NZW ≈ 192 KB @ 0x00008000
 * (FLASH base 0x00000000 as used by Qingke / this project).
 *
 * Layout:
 *   FLASH (32K ZW):
 *     .init, .vector, handle_reset, IRQ stubs, SystemInit,
 *     FULL VQF 1 kHz hot call graph + IMU/I2C/timebase + soft-float/libm
 *     needed by updateGyr/updateAcc, plus .text.hot (vqf_sample_step).
 *   FLASH_NZW (192K):
 *     cold VQF (init/mag/setters), printf/main shell, remaining libc
 *
 * Note: this toolchain's ld (binutils 2.44) rejects MEMORY region-list
 * overflow (`>FLASH FLASH_NZW`); we place explicitly instead.
 *
 * Match order: cold vqf symbols are claimed into FLASH_NZW FIRST so the
 * later "remaining vqf.o" rule in .text_zw only pulls the hot graph.
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
  /*
   * Claim cold / init-only VQF symbols into NZW BEFORE .text_zw pulls the
   * rest of vqf.o. VMA is FLASH_NZW @ 0x8000 regardless of script order.
   */
  .text_nzw :
  {
    . = ALIGN(4);
    *(.text_nzw)
    *(.text_nzw.*)
    *(.rodata_nzw)
    *(.rodata_nzw.*)

    /* Cold VQF only (init-time / mag / getters unused on 6DOF hot path) */
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

    /* VQF cold rodata (kept with cold text for locality) — hot rodata still OK in NZW */
    . = ALIGN(4);
  } >FLASH_NZW AT>FLASH_NZW

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
   * Zero-wait: reset / IRQ / SystemInit + FULL 1 kHz VQF hot graph +
   * IMU/I2C/timebase + soft-float / libm used by updateGyr/updateAcc.
   */
  .text_zw :
  {
    . = ALIGN(4);
    *(.text.handle_reset)
    *(.text.vector_handler)
    *(.text.SystemInit)
    *(.text.SysTick_Handler)
    *(.text.hot)
    *(.text.hot.*)

    /* Remaining vqf.o text = hot call graph (cold claimed above) */
    build/Middleware/vqf-c/vqf.o(.text .text.*)
    */vqf.o(.text .text.*)
    */*/vqf.o(.text .text.*)
    */*/*/vqf.o(.text .text.*)

    /* IMU / I2C / timebase / UART binary / CAN TX (high-rate) */
    *(.text.lsm6dsv_read*)
    *(.text.platform_i2c*)
    *(.text.platform_millis*)
    *(.text.platform_delay*)
    *(.text.platform_uart_write_bytes*)
    *(.text.platform_uart_send_euler*)
    *(.text.platform_can_send*)
    *(.text.millideg_i16*)
    *(.text.systick*)

    /*
     * Soft-float + save-restore from libgcc (VQF is -fno-hard-float).
     * Prefer common sf3/df3 members; full libgcc .text* is acceptable if
     * ZW headroom remains (G6U6 has 32K).
     */
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

    /* libm / string helpers reached from updateGyr / updateAcc / filterVec */
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

    /* Also match via libm.a if the toolchain splits math out of libc */
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

    . = ALIGN(4);
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
