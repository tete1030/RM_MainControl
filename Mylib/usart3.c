#include "main.h"

/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/
unsigned char Auto_Aim_Cmd=100;
void USART3_Configuration(void)
{
    USART_InitTypeDef usart3;
	GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
	
	gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&gpio);

	usart3.USART_BaudRate = 115200;
	usart3.USART_WordLength = USART_WordLength_8b;
	usart3.USART_StopBits = USART_StopBits_1;
	usart3.USART_Parity = USART_Parity_No;
	usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3,&usart3);

    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART3,ENABLE);
    
    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void USART3_SendChar(unsigned char b)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
	USART_SendData(USART3,b);
}

int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3, (uint8_t)ch);
    return ch;
}


uint8_t usart3_count,usart3_flag;
float angle_dif;

unsigned char Rx_data = 0;
unsigned char Serial_Buffer[30];//Add 8/12
static int Frame_Begin=0,Frame_End=0,X_Position=700,Y_Position=700;
char Shoot_Command =0;
int speed,Gim_yaw_tmp,Gim_pitch_tmp;

void USART3_IRQHandler(void)
{
	
		static int i=0;
		int j=0;

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART3,USART_IT_RXNE);
				Rx_data  = USART3->DR;
	//Receive Parts
			if(Rx_data=='n')
			{
				Frame_Begin=1;//'n'=110
				i=0;
			}
			if(Frame_Begin==1) 
				Serial_Buffer[i++]=Rx_data;
			if(Rx_data=='x')//'x'=120
			{
				if(Frame_Begin==1)
				{
					Frame_Begin=0;
					Frame_End=1;
		//			for (j=0;j<30;j++)
		//			Serial_Buffer[j]=0;
					i=0;
				}
			}	
			if(i>=30)
						i=0;
			
	//Handle  Parts

			if (Frame_End==1)//帧尾，处理数据
			{
				switch(Serial_Buffer[1])
				{
				
					case '1':
					{
						Auto_Aim_Cmd=1;
						Shoot_Command=1;
					
						break;
					}
					case '2'://右转
					{
						Auto_Aim_Cmd=2;
						break;
					}
					case '3':
					{
						Auto_Aim_Cmd=3;
						break;
					}
					case '4'://Auto_Scan
					{
						Auto_Aim_Cmd=4;
						Shoot_Command=0;
						
					}
					default:
					{
						Shoot_Command=0;
					
						break;
					}
				}
				Frame_End=0;
				X_Position =Serial_Buffer[2]*100+Serial_Buffer[3];
				Gim_yaw_tmp=Aimed_Position_Yaw_PID(X_Position,320);
				
				Y_Position =Serial_Buffer[4]*100+Serial_Buffer[5];
				Gim_pitch_tmp=Aimed_Position_Pitch_PID(Y_Position,240);
			}
			LED_RED_TOGGLE();
    }
}
