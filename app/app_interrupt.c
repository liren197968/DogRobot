#include "stdint.h"
#include "fsl_adc.h"
#include "app_interrupt.h"

volatile uint32_t   dwSysTicks = 0;
volatile uint32_t   MsTicks = 0;
volatile uint32_t   TempTicks = 0;

void SysTick_Handler(void)
{
    dwSysTicks++;
    TempTicks++;

    if(TempTicks >= 1000)
    {
        TempTicks = 0;
        MsTicks++;
    }
}

uint32_t HalGetUsTicks(void)
{
    return dwSysTicks;
}

uint32_t HalGetMsTicks(void)
{
    return MsTicks;
}

void HalDelayUs(uint32_t Delay)
{
    uint32_t        TickStart = 0;

    TickStart = HalGetUsTicks();

    while((HalGetUsTicks() - TickStart) < Delay)
    {

    }
}

void HalDelayMs(uint32_t Delay)
{
    uint32_t        TickStart = 0;

    TickStart = HalGetMsTicks();

    while((HalGetMsTicks() - TickStart) < Delay)
    {

    }
}

// end file
