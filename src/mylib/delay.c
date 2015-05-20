#include "stm32f4xx.h"
#include "delay.h"
#include "ticker.h"

static uint32_t fac_ms = 0;

void delay_init()
{
	fac_ms = Ticker_Get_MS_Tickcount();

}

void delay_us(uint32_t nus)
{
	uint64_t curtick = Ticker_Get_Tick();
	uint64_t endtick = curtick + ((uint64_t)nus * (uint64_t)fac_ms) / 1000;

	while(1)
	{
		if(Ticker_Get_Tick() >= endtick)
			break;
	}

}

void delay_ms(uint16_t nms)
{
	uint64_t curtick = Ticker_Get_Tick();
	uint64_t endtick = curtick + (uint64_t)nms * (uint64_t)fac_ms;

	while(1)
	{
		if(Ticker_Get_Tick() >= endtick)
			break;
	}

}

