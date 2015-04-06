#include "main.h"
#include "receiver.h"
#include "usart3.h"
#define receiver_gap 5

/*-----USART2_RX-----PA3----*/ 
//for D-BUS
uint16_t movespeed_1 = 1024;
uint16_t movespeed_2 = 0;
float pitch_propotion;
int16_t Gim_yaw_temp;

unsigned char sbus_rx_buffer[18];

void Receiver_Configuration(void)
{
		USART_InitTypeDef usart2;
		GPIO_InitTypeDef  gpio;
		NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef   dma;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3 ,GPIO_AF_USART2);

		gpio.GPIO_Pin = GPIO_Pin_3 ;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA,&gpio);

		USART_DeInit(USART2);
		usart2.USART_BaudRate = 100000;   //SBUS 100K baudrate
		usart2.USART_WordLength = USART_WordLength_8b;
		usart2.USART_StopBits = USART_StopBits_1;
		usart2.USART_Parity = USART_Parity_Even;
		usart2.USART_Mode = USART_Mode_Rx;
		usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART2,&usart2);

		USART_Cmd(USART2,ENABLE);
		USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);  

		nvic.NVIC_IRQChannel = DMA1_Stream5_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);

		DMA_DeInit(DMA1_Stream5);
		dma.DMA_Channel= DMA_Channel_4;
		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
		dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
		dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
		dma.DMA_BufferSize = 18;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Circular;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		dma.DMA_MemoryBurst = DMA_Mode_Normal;
		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream5,&dma);

		DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Stream5,ENABLE);
}

int radio_ahead_back_data=0;
int radio_left_right_data=0;
int radio_turn_data=0;
    
#define TAKE_BIT_HI(src,nbit) ((src)>>(8-(nbit)))
#define TAKE_BIT_LO(src,nbit) ((src)&(~(0xFF<<(nbit))))

uint16_t  sbus_channel_temp[15] = {0};  //  temp sbus decode channel data
uint16_t  radio_yuntai_temp[4] = {0};  //  
uint16_t  Move_Speed_X=1024, Move_Speed_Y=1024,Rotate=1024;
uint16_t  Move_Straight,Move_Horizontal;


void DMA1_Stream5_IRQHandler(void)
{
	extern int16_t Remoter_CH0_Value;
	extern int16_t Remoter_CH1_Value;
	extern int16_t Remoter_CH2_Value;
	extern int16_t Remoter_CH3_Value;
	
	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
  {
       int16_t target_speed,current_speed;
        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);


        sbus_channel_temp[0] = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; // ch0
        sbus_channel_temp[1] = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //ch1
        sbus_channel_temp[2] = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff; //ch2
        sbus_channel_temp[3] = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //ch3
        sbus_channel_temp[4] = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;//radio_switch_left
        sbus_channel_temp[5] = ((sbus_rx_buffer[5] >> 4)& 0x0003);//radio_switch_right
       
        sbus_channel_temp[6] = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);// Û±ÍX÷·
        sbus_channel_temp[7] = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);// Û±ÍY÷·
        sbus_channel_temp[8] = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);// Û±ÍZ÷·
        sbus_channel_temp[9] = sbus_rx_buffer[12];// Û±Í◊Ûº¸
        sbus_channel_temp[10] = sbus_rx_buffer[13];// Û±Í”“º¸
        sbus_channel_temp[11] = sbus_rx_buffer[14];// | (sbus_rx_buffer[15] << 8);//º¸≈Ã
        sbus_channel_temp[12] = sbus_rx_buffer[16] | (sbus_rx_buffer[17] << 8);//NULL

        
        if((sbus_channel_temp[4] != 1)&&(sbus_channel_temp[4] != 2)&&(sbus_channel_temp[4] != 3))
        {
            delay_ms(1);
					  return;
						// TODO: Fault Process
            //reset CPU
            //__set_FAULTMASK(1);
            //NVIC_SystemReset();
        }
        if((sbus_channel_temp[5] != 1)&&(sbus_channel_temp[5] != 2)&&(sbus_channel_temp[5] != 3))
        {
            delay_ms(1);
					  return;
						// TODO: Fault Process
            //reset CPU
            //__set_FAULTMASK(1);
            //NVIC_SystemReset();
        }

//if ok, then process the receive data.				
/*
//Move Part
				
				if(sbus_channel_temp[4] == 3)   //◊Û±ﬂ—°ƒ£ Ω
        {
					  //shift ∞¥º¸
            if(sbus_channel_temp[11]&0x10)
            {
               target_speed=300; 
														 
            }
            else
            {
                target_speed = 600;

            }
						//w,s∞¥º¸
            if(sbus_channel_temp[11]&0x01)
            {
							  current_speed=target_speed; 
							  movespeed_1 = movespeed_1+(float)(current_speed-movespeed_1)/10;
                Move_Speed_Y = movespeed_1+1024;            
            }
            else if(sbus_channel_temp[11]&0x02)
            {
							  current_speed=target_speed; 
							  movespeed_1 = movespeed_1+(float)(current_speed-movespeed_1)/10;
								Move_Speed_Y = -movespeed_1+1024;
            }
            else
            {
							  current_speed=0; 
							  movespeed_1 = movespeed_1+(float)(current_speed-movespeed_1)/10;
							if (Move_Speed_Y>1029)
								 	Move_Speed_Y = 1024+movespeed_1;
							else if(Move_Speed_Y<1019)
								  Move_Speed_Y = 1024-movespeed_1;
							else Move_Speed_Y = 1024;
            }
            //a,d ≤Ÿ◊˜◊Û”“
            if(sbus_channel_temp[11]&0x04)
            {
							  current_speed=target_speed; 
							  movespeed_2 = movespeed_2+(float)(current_speed-movespeed_2)/10;
                Move_Speed_X = -movespeed_2+1024;
            }
            else if(sbus_channel_temp[11]&0x08)
            {
							  current_speed=target_speed; 
							  movespeed_2 = movespeed_2+(float)(current_speed-movespeed_2)/10;
                Move_Speed_X = movespeed_2+1024;
            }
            else
            {
							  current_speed=0; 
							  movespeed_2 = movespeed_2+(float)(current_speed-movespeed_2)/10;
                Move_Speed_X = 1024;
							  if (Move_Speed_X>1029)
								 	Move_Speed_X = 1024+movespeed_2;
							else if (Move_Speed_X<1019)
								  Move_Speed_X = 1024-movespeed_2;
							else Move_Speed_X = 1024;
            }    

       }
			
			else if(sbus_channel_temp[4] == 1||sbus_channel_temp[4] ==2 )
				 {				
					Move_Speed_X=sbus_channel_temp[0];			
					Move_Speed_Y=sbus_channel_temp[1];
					Rotate=sbus_channel_temp[2];
				}

//“£øÿ∆˜
		if(sbus_channel_temp[4] == 1)
		{
				Gim_yaw=sbus_channel_temp[2];//∏¯πÃ∂®yawµ◊≈Ã
				//SendGimbalPosition(Gim_yaw,Gim_pitch,Shoot,Mode);

		}		
		else if(sbus_channel_temp[4] )
		{			
				Gim_yaw_temp = 8*(int16_t)(sbus_channel_temp[6])+1024;// Û±Íx
				
				Gim_yaw=Gim_yaw+(float)(Gim_yaw_temp-Gim_yaw)/5;
				if(Gim_yaw<256) Gim_yaw=256;
				else  if(Gim_yaw>1792) Gim_yaw=1792;
				
				pitch_propotion=((float)(768)-(float)(abs(Gim_yaw-1024)))/768;
				pitch_propotion=pitch_propotion*pitch_propotion;
				
				Gim_pitch = -7*(int16_t)(sbus_channel_temp[7])*pitch_propotion+1024;// Û±Íy
				if(Gim_pitch<512) Gim_pitch=512;
				else  if(Gim_pitch>1536) Gim_pitch=1536;
				Shoot = sbus_rx_buffer[12] ;
				//”“º¸º”»Î811
				if (sbus_rx_buffer[13])
				{
						TIM1->CCR1=800;     
						TIM1->CCR2=800;				   
												 //no.1		800-1600
												 //no.2		650-1450
												 //no.3   1450-2100
						
			//						switch (Auto_Aim_Cmd)
			//						{
			//						case 1:
			//							Gim_yaw=1024;
			//							Gim_pitch = Gim_pitch_tmp+1024;
			//						break;
			//						case 2://2 Right ,when locate in right ,Gim_yaw_tmp is negative
			//							Gim_yaw=-Gim_yaw_tmp+1024;					
			//							Gim_pitch = Gim_pitch_tmp+1024;
			//						break;
			//						case 3:
			//							Gim_yaw=-Gim_yaw_tmp+1024;
			//							Gim_pitch = Gim_pitch_tmp +1024;
			//						break;
			//						default:
			//						{
			//							Gim_yaw=1024;
			//							Gim_pitch=1024;
			//						}
			//						break;
			//						}
							
				}
				else
				{
						TIM1->CCR1=1600;
						TIM1->CCR2=1600;
				}
				
				
				SendGimbalPosition_mouse(Gim_yaw,Gim_pitch,Shoot,Mode);
				//Add 
				if(sbus_channel_temp[2]!=1024)
					Gim_yaw=sbus_channel_temp[2];
			}
	 
			if(abs(Gim_yaw-1024)<receiver_gap) Gim_yaw=1024;
			if(abs(Gim_pitch-1024)<receiver_gap) Gim_pitch=1024; 
			if(abs(Move_Speed_X-1024)<receiver_gap) Move_Speed_X=1024;
			if(abs(Move_Speed_Y-1024)<receiver_gap) Move_Speed_Y=1024; 
			Conmmunication(Move_Speed_X,Move_Speed_Y,Rotate);
					
			*/		

			Remoter_CH0_Value = sbus_channel_temp[0];
			Remoter_CH1_Value = sbus_channel_temp[1];
			Remoter_CH2_Value = sbus_channel_temp[2];
			Remoter_CH3_Value = sbus_channel_temp[3];
		}
}
		
float vP_yaw=0.7,vI_yaw=0.01,vD_yaw=0.4;
int Aimed_Position_Yaw_PID(int current_pos,int desired_pos)
{
	
    float error,last_error,error_d;
	  int output;
	  static float error_i;
	  
	  error=desired_pos-current_pos;
	  error_i+=error;
	  if(error_i>100)
			error_i=100;
		if(error_i<-100) 
			error_i=-100;
	  error_d=error-last_error;
	  last_error=error;
	  output=vP_yaw*error+vI_yaw*error_i+vD_yaw*error_d;
   	  if(output>200)
			output=200;
		if(output<-200) 
			output=-200;
	 return output;
}

float vP_pitch=0.7,vI_pitch=0,vD_pitch=0.01;
int Aimed_Position_Pitch_PID(int current_pos,int desired_pos)
{
	
    float error,last_error,error_d;
	  int output;
	  static float error_i;
	  
	  error=desired_pos-current_pos;
	  error_i+=error;
	  if(error_i>100)
			error_i=100;
		if(error_i<-100) 
			error_i=-100;
	  error_d=error-last_error;
	  last_error=error;
	  output=vP_pitch*error+vI_pitch*error_i+vD_pitch*error_d;
   	  if(output>200)
			output=200;
		if(output<-200) 
			output=-200;
	 return output;
}

