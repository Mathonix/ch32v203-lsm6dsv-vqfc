#ifndef CH32V20X_IT_H
#define CH32V20X_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
