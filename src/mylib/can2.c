#include "stm32f4xx.h"
#include "receiver.h"
#include "stm32f4xx_it.h"

/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/
int16_t Gim_yaw;
int16_t Gim_pitch;
uint8_t Shoot;

uint8_t Mode;

void CAN2_Configuration(void)
{
	CAN_InitTypeDef can;
	CAN_FilterInitTypeDef can_filter;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpio);

	nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = ITP_CAN2_RX0_PREEMPTION;
	nvic.NVIC_IRQChannelSubPriority = ITP_CAN2_RX0_SUB;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	CAN_DeInit(CAN2);
	CAN_StructInit(&can);

	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = DISABLE;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = DISABLE;
	can.CAN_RFLM = DISABLE;
	can.CAN_TXFP = ENABLE;
	can.CAN_Mode = CAN_Mode_Normal;
	can.CAN_SJW = CAN_SJW_1tq;
	can.CAN_BS1 = CAN_BS1_9tq;
	can.CAN_BS2 = CAN_BS2_4tq;
	can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN2, &can);

	can_filter.CAN_FilterNumber = 14;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = 0x601 << 5;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0xFFFF;
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = 0; //the message which pass the filter save in fifo0
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);

	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}

int32_t angle;

void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg Rx_message;
	if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_Receive(CAN2, CAN_FIFO0, &Rx_message);

		angle = (int32_t) ((uint32_t) (Rx_message.Data[0]) << 24
				| (uint32_t) (Rx_message.Data[1]) << 16
				| (uint32_t) (Rx_message.Data[2]) << 8
				| (uint32_t) (Rx_message.Data[3]) << 0);
	}
}

void SendGimbalPosition(uint16_t Gim_yaw, uint16_t Gim_pitch, uint8_t Shoot,
		uint8_t Mode)
{
	CanTxMsg tx2_message;

	tx2_message.StdId = 0x402;
	tx2_message.DLC = 0x08;
	tx2_message.RTR = CAN_RTR_Data;
	tx2_message.IDE = CAN_Id_Standard;

	tx2_message.Data[0] = (sbus_channel_temp[2] & 0xff00) >> 8;
	;
	tx2_message.Data[1] = (sbus_channel_temp[2] & 0x00ff) >> 0;
	tx2_message.Data[2] = (sbus_channel_temp[3] & 0xff00) >> 8;   //Pitch
	tx2_message.Data[3] = (sbus_channel_temp[3] & 0x00ff) >> 0;
	tx2_message.Data[4] = (sbus_channel_temp[5] & 0xff00) >> 8;   //Shoot
	tx2_message.Data[5] = (sbus_channel_temp[5] & 0x00ff) >> 0;
	tx2_message.Data[6] = 0;
	tx2_message.Data[7] = 0;

	CAN_Transmit(CAN2, &tx2_message);
	CAN_Transmit(CAN1, &tx2_message);   //Send to  shoot Control System

}

void SendGimbalPosition_mouse(int16_t Gim_yaw, int16_t Gim_pitch, uint8_t Shoot,
		uint8_t Mode)
{
	CanTxMsg tx2_message;

	tx2_message.StdId = 0x402;
	tx2_message.DLC = 0x08;
	tx2_message.RTR = CAN_RTR_Data;
	tx2_message.IDE = CAN_Id_Standard;

	tx2_message.Data[0] = (Gim_yaw & 0xff00) >> 8;
	;
	tx2_message.Data[1] = (Gim_yaw & 0x00ff) >> 0;
	tx2_message.Data[2] = (Gim_pitch & 0xff00) >> 8;   //Pitch
	tx2_message.Data[3] = (Gim_pitch & 0x00ff) >> 0;
	if (Shoot == 1)
	{ //tx2_message.Data[4] = (sbus_channel_temp[4]&0xff00)>>8;//Shoot
		tx2_message.Data[5] = 1;
	}
	else
		tx2_message.Data[5] = 3;

	tx2_message.Data[6] = 0;
	tx2_message.Data[7] = 0;

	CAN_Transmit(CAN2, &tx2_message);
	CAN_Transmit(CAN1, &tx2_message); //Send to  shoot Control System

}
