#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "can1.h"
#include "debug.h"
#include <stdio.h>


/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/
void (*rh)(CanRxMsg*);
void (*sh)(uint16_t, int8_t);

#define PACKET_QUEUE_SIZE 10

uint16_t mailbox0_id=0, mailbox1_id=0, mailbox2_id=0;

typedef struct Packet_Queue
{
	uint16_t id;
	uint16_t addr;
	int8_t data[8];
	uint8_t size;
} Packet_Queue;
Packet_Queue pq[PACKET_QUEUE_SIZE];
uint8_t pq_start = 0;
uint8_t pq_end = 0;
int8_t pq_full = 0;

uint32_t async_transmit_times = 0;
uint32_t queue_full_times = 0;


int8_t mutex_transmit = 0;
int8_t mutex_queue = 0;

void CAN1_Configuration(void (*send_handler)(uint16_t, int8_t), void (*receive_handler)(CanRxMsg*), uint16_t IdHigh, uint16_t IdHighMask, uint16_t IdLow, uint16_t IdLowMask)
{
	CAN_InitTypeDef can;
	CAN_FilterInitTypeDef can_filter;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);


	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio);

	rh = receive_handler;
	sh = send_handler;
	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = ITP_CAN1_RX0_PREEMPTION;
	nvic.NVIC_IRQChannelSubPriority = ITP_CAN1_RX0_SUB;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = ITP_CAN1_TX_PREEMPTION;
	nvic.NVIC_IRQChannelSubPriority = ITP_CAN1_TX_SUB;
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
	can.CAN_SJW = CAN_SJW_1tq;
	can.CAN_BS1 = CAN_BS1_9tq;
	can.CAN_BS2 = CAN_BS2_4tq;
	// TODO: Set Properiate Prescaler
	can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber = 0;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_16bit;
	can_filter.CAN_FilterIdHigh = IdHigh << 5;
	can_filter.CAN_FilterIdLow = IdLow << 5;
	can_filter.CAN_FilterMaskIdHigh = IdHighMask << 5;
	can_filter.CAN_FilterMaskIdLow = IdLowMask << 5;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);

	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);
}

void CAN1_TX_IRQHandler(void)
{
	uint16_t id;
	int8_t code;
	if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
	{
		if(sh)
		{
			if(CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1))
			{
				if(CAN1->TSR & CAN_TSR_TME0)
				{
					id = mailbox0_id;
					code = ((CAN1->TSR & CAN_TSR_TXOK0) == CAN_TSR_TXOK0);
				}
				else
				{
					id = mailbox1_id;
					code = ((CAN1->TSR & CAN_TSR_TXOK1) == CAN_TSR_TXOK1);
				}
			}
			else
			{
				id = mailbox2_id;
				code = ((CAN1->TSR & CAN_TSR_TXOK2) == CAN_TSR_TXOK2);
			}
		}
		CAN_ClearITPendingBit(CAN1, CAN_IT_TME);

		if(sh)
			sh(id, code);

		if(mutex_transmit == 0 && mutex_queue == 0 && (pq_full == 1 || pq_end != pq_start))
		{
			CAN1_Transmit(pq[pq_start].id, pq[pq_start].addr, pq[pq_start].data, pq[pq_start].size);
			pq_full = 0;
			pq_start = (pq_start + 1) % PACKET_QUEUE_SIZE;
		}
	}
}

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;

	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		if(rh)
			rh(&rx_message);
	}
}

int8_t CAN1_Transmit(uint16_t id, uint16_t addr, char* data, uint8_t size)
{
	CanTxMsg Tx_message;
	uint8_t used_mailbox;
	uint8_t i;
	Tx_message.StdId = addr;
	Tx_message.IDE = CAN_Id_Standard;
	Tx_message.RTR = CAN_RTR_Data;
	Tx_message.DLC = size;
	for (i = 0; i < size; i++)
		Tx_message.Data[i] = data[i] & 0xff;

	mutex_transmit = 1;
	used_mailbox = CAN_Transmit(CAN1, &Tx_message);
	if (used_mailbox == CAN_TxStatus_NoMailBox)
	{
		if(sh)
			sh(id, 0);
		printf("[CAN1_Transmit] No mailbox\r\n");
		return -1;
	}
	switch(used_mailbox)
	{
	case 0:
		mailbox0_id = id;
		break;
	case 1:
		mailbox1_id = id;
		break;
	case 2:
		mailbox2_id = id;
		break;
	}
	mutex_transmit = 0;
	return 1;
}

int8_t CAN1_AsyncTransmit(uint16_t id, uint16_t addr, char* data, uint8_t size)
{
	int ret;
	if(CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2))
	{
		if((ret = CAN1_Transmit(id, addr, data, size)) == 1)
			return 1;
		else if(ret == -2)
			return -2;
	}

	mutex_queue = 1;
	async_transmit_times ++;
	if(pq_full == 0)
	{
		pq[pq_end].id = id;
		pq[pq_end].addr = addr;
		pq[pq_end].size = size;
		pq[pq_end].data[0] = data[0];
		pq[pq_end].data[1] = data[1];
		pq[pq_end].data[2] = data[2];
		pq[pq_end].data[3] = data[3];
		pq[pq_end].data[4] = data[4];
		pq[pq_end].data[5] = data[5];
		pq[pq_end].data[6] = data[6];
		pq[pq_end].data[7] = data[7];
		pq_end = (pq_end + 1) % PACKET_QUEUE_SIZE;
		if(pq_start == pq_end) pq_full = 1;
	}
	else
	{
		queue_full_times ++;
		mutex_queue = 0;
		if(sh)
			sh(id, 0);
		printf("[CAN1_AsyncTransmit] Queue Full\r\n");
		return -1;
	}
	mutex_queue = 0;
	return 0;

}

