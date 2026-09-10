/**
 * Clock init lives in Platform/platform_ch32v203.c (SystemInit).
 * This file keeps MRS-compatible symbols.
 */
#include "system_ch32v20x.h"

void SystemCoreClockUpdate(void)
{
    /* Platform sets SystemCoreClock in SystemInit; nothing to recompute here. */
}
