#include "main.h"

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/



uint8_t Sensor_Data[4][8]={0};
void CAN1_Configuration(void)
{
		CAN_InitTypeDef        can;
		CAN_FilterInitTypeDef  can_filter;
		GPIO_InitTypeDef       gpio;
		NVIC_InitTypeDef       nvic;
		int16_t std_id=0x404;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);

		gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &gpio);

		nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 2;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);

		nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);    

		CAN_DeInit(CAN1);
		CAN_StructInit(&can);

		can.CAN_TTCM = DISABLE;
		can.CAN_ABOM = DISABLE;
		can.CAN_AWUM = DISABLE;
		can.CAN_NART = DISABLE;
		can.CAN_RFLM = DISABLE;
		can.CAN_TXFP = ENABLE;
		can.CAN_Mode = CAN_Mode_Normal;
		can.CAN_SJW  = CAN_SJW_1tq;
		can.CAN_BS1 = CAN_BS1_9tq;
		can.CAN_BS2 = CAN_BS2_4tq;
		// TODO: Set Properiate Prescaler
		can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
		CAN_Init(CAN1, &can);

		can_filter.CAN_FilterNumber=0;
		can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
		can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
		can_filter.CAN_FilterIdHigh=std_id<<5;
		can_filter.CAN_FilterIdLow=0x0000;
		can_filter.CAN_FilterMaskIdHigh=0x8000;
		can_filter.CAN_FilterMaskIdLow=0x0000;
		can_filter.CAN_FilterFIFOAssignment=0;
		can_filter.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&can_filter);

		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
		CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

void CAN1_TX_IRQHandler(void)
{
		if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
		{
				CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		}
}

void CAN1_RX0_IRQHandler(void)
{
		CanRxMsg rx_message;
		int8_t i=0;

		if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
		{
				CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
				CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
				switch((rx_message.StdId&0x03c0)>>6)//验证是谁发上来的
				{
						case 1:
								for(i=0;i<8;i++)
										Sensor_Data[0][i]=rx_message.Data[i];
								break;
						case 2:
								for(i=0;i<8;i++)
										Sensor_Data[1][i]=rx_message.Data[i];
								break;
						case 3:
								for(i=0;i<8;i++)
										Sensor_Data[2][i]=rx_message.Data[i];
								break;
						case 4:
								for(i=0;i<8;i++)
										Sensor_Data[3][i]=rx_message.Data[i];
								break;
						case 5:
								break;
				}
				LED_RED_TOGGLE();	 	   
		}
}

void Can_Fun(int16_t addr,int16_t speed)
{
		CanTxMsg Tx_message; 
		Tx_message.StdId = CAN_CONTROL_IDENTIFIER << 3 | (addr & 0x7);
		Tx_message.IDE = CAN_Id_Standard;
		Tx_message.RTR = CAN_RTR_Data;
		Tx_message.DLC = 2;
		Tx_message.Data[0] = speed & 0xff;
	  Tx_message.Data[1] = speed >> 8;
		CAN_Transmit(CAN1,&Tx_message);
}

void Conmmunication(int16_t Move_Speed_X,int16_t Move_Speed_Y,int Rotate)
{
      CanTxMsg Tx_message; 
			uint32_t ID_each[4]={0x40,0x80,0xc0,0x100};
			int8_t i=0;
//			int16_t Negative_Speed;
		
				
			for(i=0;i<4;i++)
			{
					Tx_message.StdId = ID_each[i];
					Tx_message.IDE = CAN_Id_Standard;
					Tx_message.RTR = CAN_RTR_Data;
					Tx_message.DLC = 0x08;
				
					Tx_message.Data[0] = (Move_Speed_X&0xff00)>>8;
					Tx_message.Data[1] = Move_Speed_X&0x00ff;
					Tx_message.Data[2] = (Move_Speed_Y&0xff00)>>8;
					Tx_message.Data[3] = Move_Speed_Y&0x00ff;
					Rotate=Gim_yaw;
//					if(abs(angle)<50) Rotate=1024;
//					else if	(abs(angle)>50) Rotate=angle/2+1024;
//						if (Rotate<364) Rotate=364;
//						if (Rotate>1684) Rotate=1684;
					Tx_message.Data[4] = (Rotate&0xff00)>>8;;
					Tx_message.Data[5] = Rotate&0x00ff;;
					Tx_message.Data[6] = 0x06;
					Tx_message.Data[7] = 0x07;
				  CAN_Transmit(CAN1,&Tx_message);
					delay_ms(1);
			
			}
				
	
}

void  Auto_Aim()
{
}

