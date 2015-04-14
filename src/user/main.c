#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "led.h"
#include "driver.h"
#include "usart3.h"
#include "delay.h"
#include "main.h"
#include "diag/Trace.h"

char keys_down[9];
char keys[] =
{ 'W', 'S', 'A', 'D', 'Q', 'E', '^', '*' };
extern uint16_t sbus_channel_temp[];

int16_t Remoter_CH0_Value = 0;
int16_t Remoter_CH1_Value = 0;
int16_t Remoter_CH2_Value = 0;
int16_t Remoter_CH3_Value = 0;

void Remoter_Test()
{
	while (1)
	{
		/*
		 //printf("ch0=%d, ch1=%d, ch2=%d, ch3=%d, s1=%d, s2=%d \r\n", sbus_channel_temp[0], sbus_channel_temp[1],sbus_channel_temp[2],sbus_channel_temp[3],sbus_channel_temp[4],sbus_channel_temp[5]);
		 
		 j=0;
		 for(i=0; i<8; i++)
		 {
		 if((sbus_channel_temp[11]>>i) & 1)
		 {
		 keys_down[j++] = keys[i];
		 }
		 }
		 keys_down[j] = 0;
		 
		 
		 printf("mouse_x=%d, mouse_y=%d, mouse_z=%d, mouse_l=%d, mouse_r=%d, key=%s \r\n\r\n", sbus_channel_temp[6], sbus_channel_temp[7], sbus_channel_temp[8], sbus_channel_temp[9], sbus_channel_temp[10], keys_down);
		 */
	}
}

int main(void)
{
	Led_Configuration();
	//USART2_Configuration(); 
	//PWM_Configuration();
	//USART3_Configuration();//接受摄像头传回的串口信号
	Driver_Configuration();
	Driver_Enable(
			DRIVER_LEFT_FRONT | DRIVER_RIGHT_FRONT | DRIVER_LEFT_END
					| DRIVER_RIGHT_END);
	//CAN2_Configuration();	
	USART3_Configuration();		//配置UART
	//Receiver_Configuration();
	trace_printf("Start;\r\n");
	while (1)
	{
		delay_ms(1000);
		//printf("CH2=%d, CH3=%d\r\n", Remoter_CH2_Value, Remoter_CH3_Value);

		//vx = (((float)Remoter_CH3_Value - 1024) * CAR_MAX_SPEED) / 660;
		//vy = (((float)Remoter_CH2_Value - 1024) * CAR_MAX_SPEED) / 660;
		//w0 = (((float)Remoter_CH0_Value - 1024) * PI) / 660;
		Driver_Set_Speed(3, 0, 0);

		LED_GREEN_TOGGLE();
		LED_RED_TOGGLE();
	}

	while (1)
		;

}
