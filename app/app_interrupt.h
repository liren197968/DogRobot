#ifndef _APP_INTERRUPT_H_
#define _APP_INTERRUPT_H_

#include "stdint.h"

void HalDelayUs(uint32_t Delay);
void HalDelayMs(uint32_t Delay);

extern volatile uint32_t dwSysTicks;

extern void SysTick_Handler(void);

#endif

// end file
