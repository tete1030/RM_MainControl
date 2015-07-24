#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "led.h"
#include "driver.h"
#include "usart3.h"
#include "delay.h"
#include "main.h"
#include "diag/Trace.h"
#include "receiver.h"
#include "debug.h"
#include "buzzer.h"
#include "ticker.h"
#include "gimbal.h"
#include "execute.h"

volatile uint8_t control_enable = 0;

void Maincontrol_Set_Enable_Control(uint8_t enable)
{
	control_enable = enable;
	BUZZER_ON();
	delay_ms(200);
	BUZZER_OFF();
}

uint8_t Maincontrol_Get_Enable_Control_State(void)
{
	return control_enable;
}

int main(void)
{
	uint i;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Ticker_Configuration();
	delay_init();

	Led_Configuration();

    Buzzer_Configuration();
	USART3_Configuration();

	Execute_Init();

	delay_ms(100);

	Driver_Configuration(); // 使用CAN1，已在内部初始化CAN1
	Driver_Set_Enable( DRIVER_ALL, DRIVER_ALL );

    Gimbal_Configuration(); // 使用CAN2，一定要先初始化CAN1再初始化CAN2过滤器

	//Debug_Configuration();
	Receiver_Configuration();

	LED_GREEN_ON();
	i = 0;
	while (1)
	{
		delay_us(500);
		if(i == 400)
		{
			LED_RED_TOGGLE();
			i = 0;
		}

		//Debug_Execute_Computer_Command();

		Execute_Do_Receiver_Command();

		i++;
	}

}
