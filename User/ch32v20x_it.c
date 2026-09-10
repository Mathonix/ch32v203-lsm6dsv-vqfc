#include "ch32v20x_it.h"

void NMI_Handler(void)
{
    while (1) {
    }
}

void HardFault_Handler(void)
{
    while (1) {
    }
}

void SysTick_Handler(void)
{
    /* Timebase is polled from free-running SysTick in Platform/; IRQ unused. */
}
