#include <stdint.h>
#include <stdio.h>
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

char keys_down[9];
char keys[] =
{ 'W', 'S', 'A', 'D', 'Q', 'E', '^', '*' };
extern uint16_t sbus_channel_temp[];

int16_t Remoter_CH0_Value = 1024;
int16_t Remoter_CH1_Value = 1024;
int16_t Remoter_CH2_Value = 1024;
int16_t Remoter_CH3_Value = 1024;


int main(void)
{
	int16_t vx, vy;
	float w0;

	Led_Configuration();
	//PWM_Configuration();
	USART3_Configuration();

	delay_ms(50);
	Driver_Configuration();
	Driver_Set_Enable( DRIVER_ALL, DRIVER_ALL );
	//CAN2_Configuration();

	//Debug_Configuration();
	Receiver_Configuration();

	Buzzer_Configuration();

	LED_RED_ON();
	while(1)
	{
		LED_RED_TOGGLE();
		LED_GREEN_TOGGLE();
		BUZZER_TOGGLE();
		delay_ms(50);
	}
	//printf("Start;\r\n");
	while (1)
	{
		delay_ms(50);
		//printf("CH2=%d, CH3=%d\r\n", Remoter_CH2_Value, Remoter_CH3_Value);

		vx = (((float)Remoter_CH3_Value - 1024) * CAR_MAX_SPEED) / 660;
		vy = (((float)Remoter_CH2_Value - 1024) * CAR_MAX_SPEED) / 660;
		w0 = (((float)Remoter_CH0_Value - 1024) * PI ) / 660;
		Driver_Set_Speed(vx, vy, w0);
		//Debug_Execute_Computer_Command();

		LED_GREEN_TOGGLE();
		LED_RED_TOGGLE();
	}

}
