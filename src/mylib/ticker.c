#include "ticker.h"
#include "stm32f4xx_it.h"

#define TICKER_CYCLE_COUNT 0x01000000

// TODO: update all necessary to volatile!!!!
volatile uint64_t ticker = 0;

void Ticker_Configuration(void)
{
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = SysTick_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = ITP_SYSTICK_PREEMPTION;
    nvic.NVIC_IRQChannelSubPriority = ITP_SYSTICK_SUB;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    SysTick->LOAD = TICKER_CYCLE_COUNT - 1;
    SysTick->CTRL = (SysTick->CTRL & 0xfffffff8) | 3; // div8, interrupt, enable
}

uint64_t Ticker_Get_Tick()
{
	uint64_t ret;
	// Disable Systick
	NVIC->ICER[SysTick_IRQn >> 0x05] = (uint32_t) 0x01 << (SysTick_IRQn & (uint8_t) 0x1F);
	ret = ticker + TICKER_CYCLE_COUNT - SysTick->VAL;
    // Enable Systick
    NVIC->ISER[SysTick_IRQn >> 0x05] = (uint32_t) 0x01 << (SysTick_IRQn & (uint8_t) 0x1F);
    return ret;
}

inline uint32_t Ticker_Get_MS_Tickcount()
{
    if(SysTick->CALIB & SysTick_CALIB_NOREF_Msk)
        return SystemCoreClock / 8000;
    else
        return SysTick->CALIB & SysTick_CALIB_TENMS_Msk;
}

void SysTick_Handler(void)
{
    ticker += TICKER_CYCLE_COUNT;
}

