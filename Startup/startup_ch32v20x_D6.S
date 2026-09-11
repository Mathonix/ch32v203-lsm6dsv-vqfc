/*
 * Minimal Qingke V4 / CH32V20x_D6 startup (CH32V203G6U6 class).
 * Vector layout follows WCH CH32V20x D6 public interrupt map.
 */
  .section .init, "ax", @progbits
  .global _start
  .align 2
_start:
  j handle_reset

  .section .vector, "ax", @progbits
  .align 1
_vector_base:
  .option norvc
  .word   _start
  .word   0
  .word   NMI_Handler
  .word   HardFault_Handler
  .word   0
  .word   Ecall_M_Mode_Handler
  .word   0
  .word   0
  .word   Ecall_U_Mode_Handler
  .word   Break_Point_Handler
  .word   0
  .word   0
  .word   SysTick_Handler
  .word   0
  .word   SW_Handler
  .word   0
  /* External IRQs */
  .word   Default_Handler /* WWDG */
  .word   Default_Handler /* PVD */
  .word   Default_Handler /* TAMPER */
  .word   Default_Handler /* RTC */
  .word   Default_Handler /* FLASH */
  .word   Default_Handler /* RCC */
  .word   Default_Handler /* EXTI0 */
  .word   Default_Handler /* EXTI1 */
  .word   Default_Handler /* EXTI2 */
  .word   Default_Handler /* EXTI3 */
  .word   Default_Handler /* EXTI4 */
  .word   Default_Handler /* DMA1_CH1 */
  .word   Default_Handler /* DMA1_CH2 */
  .word   Default_Handler /* DMA1_CH3 */
  .word   Default_Handler /* DMA1_CH4 */
  .word   Default_Handler /* DMA1_CH5 */
  .word   Default_Handler /* DMA1_CH6 */
  .word   Default_Handler /* DMA1_CH7 */
  .word   Default_Handler /* ADC1_2 */
  .word   Default_Handler /* USB_HP_CAN1_TX */
  .word   Default_Handler /* USB_LP_CAN1_RX0 */
  .word   Default_Handler /* CAN1_RX1 */
  .word   Default_Handler /* CAN1_SCE */
  .word   Default_Handler /* EXTI9_5 */
  .word   Default_Handler /* TIM1_BRK */
  .word   Default_Handler /* TIM1_UP */
  .word   Default_Handler /* TIM1_TRG_COM */
  .word   Default_Handler /* TIM1_CC */
  .word   Default_Handler /* TIM2 */
  .word   Default_Handler /* TIM3 */
  .word   Default_Handler /* TIM4 */
  .word   Default_Handler /* I2C1_EV */
  .word   Default_Handler /* I2C1_ER */
  .word   Default_Handler /* I2C2_EV */
  .word   Default_Handler /* I2C2_ER */
  .word   Default_Handler /* SPI1 */
  .word   Default_Handler /* SPI2 */
  .word   Default_Handler /* USART1 */
  .word   Default_Handler /* USART2 */
  .word   Default_Handler /* USART3 */
  .word   Default_Handler /* EXTI15_10 */
  .word   Default_Handler /* RTCAlarm */
  .word   Default_Handler /* USBWakeUp */
  .word   Default_Handler /* USBHD */
  .word   Default_Handler /* USBHDWakeUp */
  .word   Default_Handler /* UART4 */
  .word   Default_Handler /* DMA1_Channel8 */

  .option rvc
  .section .text.vector_handler, "ax", @progbits
  .weak NMI_Handler
  .weak HardFault_Handler
  .weak Ecall_M_Mode_Handler
  .weak Ecall_U_Mode_Handler
  .weak Break_Point_Handler
  .weak SysTick_Handler
  .weak SW_Handler
  .weak Default_Handler

NMI_Handler:
HardFault_Handler:
Ecall_M_Mode_Handler:
Ecall_U_Mode_Handler:
Break_Point_Handler:
SysTick_Handler:
SW_Handler:
Default_Handler:
1:  j 1b

  .section .text.handle_reset, "ax", @progbits
  .weak handle_reset
  .align 1
handle_reset:
  .option push
  .option norelax
  la gp, __global_pointer$
  .option pop
  la sp, _eusrstack

  /* copy .data */
  la a0, _data_lma
  la a1, _data_vma
  la a2, _edata
  beq a1, a2, 2f
1:
  lw t0, 0(a0)
  sw t0, 0(a1)
  addi a0, a0, 4
  addi a1, a1, 4
  bltu a1, a2, 1b
2:
  /* copy .text.hot_ram (VQF-fxp 1 kHz hot path LMA → HOT_RAM VMA) */
  la a0, _shot_ram_lma
  la a1, _shot_ram
  la a2, _ehot_ram
  beq a1, a2, 2f
1:
  lw t0, 0(a0)
  sw t0, 0(a1)
  addi a0, a0, 4
  addi a1, a1, 4
  bltu a1, a2, 1b
2:
  /* copy .algo_ram (Mahony/Comp LMA → ALGO_RAM VMA) */
  la a0, _salgo_ram_lma
  la a1, _salgo_ram
  la a2, _ealgo_ram
  beq a1, a2, 2f
1:
  lw t0, 0(a0)
  sw t0, 0(a1)
  addi a0, a0, 4
  addi a1, a1, 4
  bltu a1, a2, 1b
2:
  /* clear .bss */
  la a0, _sbss
  la a1, _ebss
  beq a0, a1, 2f
1:
  sw zero, 0(a0)
  addi a0, a0, 4
  bltu a0, a1, 1b
2:
  /* mtvec = vector base | mode1 (absolute vector) */
  la t0, _vector_base
  ori t0, t0, 3
  csrw mtvec, t0

  li t0, 0x08
  csrs mstatus, t0

  jal SystemInit
  la t0, main
  csrw mepc, t0
  mret
