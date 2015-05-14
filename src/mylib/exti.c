#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

void Exti_Configuration(void)
{
	GPIO_InitTypeDef gpio;
	EXTI_InitTypeDef exti;
	NVIC_InitTypeDef nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;

	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOC, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOE, &gpio);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource6);

	exti.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line5 | EXTI_Line6;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	nvic.NVIC_IRQChannel = EXTI0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = ITP_EXTI0_PREEMPTION;
	nvic.NVIC_IRQChannelSubPriority = ITP_EXTI0_SUB;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	nvic.NVIC_IRQChannel = EXTI1_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = ITP_EXTI1_PREEMPTION;
	nvic.NVIC_IRQChannelSubPriority = ITP_EXTI1_SUB;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = ITP_EXTI9_5_PREEMPTION;
	nvic.NVIC_IRQChannelSubPriority = ITP_EXTI9_5_SUB;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0))
	{
		//Fuyang_Number = 870000;
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1))
	{
		//Fuyang_Number = 30000;
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line5))
	{
		//Xuanzhuan_Number = 2000;
		EXTI_ClearITPendingBit(EXTI_Line5);
	}

	if (EXTI_GetITStatus(EXTI_Line6))
	{
		//xXuanzhuan_Number = 78000;
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
}
