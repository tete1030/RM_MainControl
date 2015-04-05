#include "main.h"
#include "gyroscope.h"
#include "receiver.h"
char keys_down[9];
char keys[] = {'W', 'S', 'A', 'D', 'Q', 'E', '^', '*'};
extern uint16_t sbus_channel_temp[];
int16_t Fun_Speed = 0;
int32_t Fun_Speed_32 = 0;
extern void Can_Fun(int16_t);
int main(void)
{   
    int i=0,j=0;
//	  int16_t a=2,b=200;
    SystemInit(); 
    Led_Configuration();
	  //USART2_Configuration(); 
	  //PWM_Configuration();
		//USART3_Configuration();//接受摄像头传回的串口信号
	  //delay_ms(1000);
    CAN1_Configuration();
    CAN2_Configuration();	
	  USART3_Configuration();//配置UART
    Receiver_Configuration();
	  printf("Test Start;\r\n");
		while(1)
		{
			  delay_ms(200);
				printf("ch0=%d, ch1=%d, ch2=%d, ch3=%d, s1=%d, s2=%d \r\n", sbus_channel_temp[0], sbus_channel_temp[1],sbus_channel_temp[2],sbus_channel_temp[3],sbus_channel_temp[4],sbus_channel_temp[5]);
				j=0;
				for(i=0; i<8; i++)
				{
					 if((sbus_channel_temp[11]>>i) & 1)
					 {
						  keys_down[j++] = keys[i];
					 }
				}
				keys_down[j] = 0;
				Fun_Speed_32 = Fun_Speed;
				Can_Fun((int16_t)((Fun_Speed_32 - 364) * 9600 / 1320));
				//printf("mouse_x=%d, mouse_y=%d, mouse_z=%d, mouse_l=%d, mouse_r=%d, key=%s \r\n\r\n", sbus_channel_temp[6], sbus_channel_temp[7], sbus_channel_temp[8], sbus_channel_temp[9], sbus_channel_temp[10], keys_down);
		}
		
	//while(1)
    //{
     //delay_ms(500);
		 //LED_GREEN_TOGGLE();
    // Conmmunication(a,b);
    //}
}
