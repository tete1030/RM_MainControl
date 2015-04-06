#include "main.h"
#include "gyroscope.h"
#include "receiver.h"
char keys_down[9];
char keys[] = {'W', 'S', 'A', 'D', 'Q', 'E', '^', '*'};
extern uint16_t sbus_channel_temp[];

int16_t Remoter_CH0_Value = 0;
int16_t Remoter_CH1_Value = 0;
int16_t Remoter_CH2_Value = 0;
int16_t Remoter_CH3_Value = 0;

// Unit: mm
#define CAR_WHEEL_RADIUS 75
#define CAR_WIDTH 263.3
#define CAR_LENGTH 215
#define CAR_MAX_SPEED 2000.0
#define CAR_ENCODER_NUM 500
#define CAR_ENCODER_MAX_COUNT_PER_10MS 1020
#define CAR_ENCODER_MAX_COUNT_PER_SECOND (CAR_MAX_ENCODER_COUNT_PER_10MS * 100)

#define PI 3.1415




extern void Can_Fun(int16_t,int16_t);

int main(void)
{   
    int i=0,j=0;
    float vx = 0, vy = 0, w0, p;
		float wlf, wrf, wle, wre;
	  int16_t vlf, vrf, vle, vre;  
	
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
	
		p = (CAR_WIDTH +  CAR_LENGTH) / 2;
	  i=0;
		while(1)
		{
			  delay_ms(1000);
			  printf("CH2=%d, CH3=%d\r\n", Remoter_CH2_Value, Remoter_CH3_Value);
			  vx = (((float)Remoter_CH3_Value - 1024) * CAR_MAX_SPEED) / 660;
			  vy = (((float)Remoter_CH2_Value - 1024) * CAR_MAX_SPEED) / 660;
			  w0 = (((float)Remoter_CH0_Value - 1024) * PI) / 660;
				
			  // 
			  wlf = (vx + vy - p * w0) * CAR_ENCODER_NUM / (2 * PI * CAR_WHEEL_RADIUS);
			  wrf = (vx - vy + p * w0) * CAR_ENCODER_NUM / (2 * PI * CAR_WHEEL_RADIUS);
			  wle = (vx - vy - p * w0) * CAR_ENCODER_NUM / (2 * PI * CAR_WHEEL_RADIUS);
			  wre = (vx + vy + p * w0) * CAR_ENCODER_NUM / (2 * PI * CAR_WHEEL_RADIUS);
			
			  printf("v=%f, %f, %f\r\n", vx, vy, w0);
			  printf("enc=%f, %f, %f, %f\r\n", wlf, wrf, wle, wre);
			  
			  switch(i)
				{
					case 0:
						Can_Fun(CAN_DRIVER_LEFT_FRONT_ADDR , 4000);
						break;
					case 1:
						Can_Fun(CAN_DRIVER_RIGHT_FRONT_ADDR , 4000);
						break;
					case 2:
						Can_Fun(CAN_DRIVER_LEFT_END_ADDR , 4000);
						break;
					case 3:
						Can_Fun(CAN_DRIVER_RIGHT_END_ADDR , 4000);
						break;
				}
			  i=(i+1)%4;
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
		
	//while(1)
    //{
     //delay_ms(500);
		 //LED_GREEN_TOGGLE();
    // Conmmunication(a,b);
    //}
}
