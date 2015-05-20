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


Car_Status car_status;

void Car_Status_Init()
{
    memset(&car_status, 0, sizeof(car_status));
}

int8_t Car_Status_Lock()
{
    if(car_status.mutex_this == 1) return 0;
    car_status.mutex_this = 1;
    return 1;
}

void Car_Status_Unlock()
{
    car_status.mutex_this = 0;
}

int main(void)
{
	int16_t vx, vy;
	float w0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Ticker_Configuration();
	delay_init();

    Car_Status_Init();

	Led_Configuration();
    Buzzer_Configuration();
	USART3_Configuration();

    Gimbal_Configuration();
	delay_ms(50);
	Driver_Configuration();
	Driver_Set_Enable( DRIVER_ALL, DRIVER_ALL );

	//Debug_Configuration();
	Receiver_Configuration();

	LED_RED_ON();
	//printf("Start;\r\n");
	while (1)
	{
		delay_ms(50);
		//printf("CH2=%d, CH3=%d\r\n", Remoter_CH2_Value, Remoter_CH3_Value);

		//vx = (((float)Remoter_CH3_Value - 1024) * CAR_MAX_X_SPEED) / 660;
		//vy = (((float)Remoter_CH2_Value - 1024) * CAR_MAX_Y_SPEED) / 660;
		//w0 = (((float)Remoter_CH0_Value - 1024) * PI ) / 660;
		//Driver_Set_Speed(vx, vy, w0);
		//Debug_Execute_Computer_Command();
		Execute_Receiver_Command();

		LED_GREEN_TOGGLE();
		LED_RED_TOGGLE();
	}

}
