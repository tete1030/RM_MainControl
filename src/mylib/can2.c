//
// Created by Texot Qexoq on 5/16/15.
//
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "can2.h"
#include "can_packet_queue.h"

/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/
void (*can2_rh)(CanRxMsg*);
void (*can2_sh)(uint16_t, int8_t);

#define CAN2_PACKET_QUEUE_SIZE 10

uint16_t can2_mailbox0_id =0, can2_mailbox1_id =0, can2_mailbox2_id =0;

CAN_Packet_Queue can2_pq[CAN2_PACKET_QUEUE_SIZE];
uint8_t can2_pq_start = 0;
uint8_t can2_pq_end = 0;
int8_t can2_pq_full = 0;

uint32_t can2_async_transmit_times = 0;
uint32_t can2_queue_full_times = 0;

int8_t can2_mutex_transmit = 0;
int8_t can2_mutex_queue = 0;

void CAN2_Configuration(void (*send_handler)(uint16_t, int8_t), void (*receive_handler)(CanRxMsg*), uint16_t IdHigh, uint16_t IdHighMask, uint16_t IdLow, uint16_t IdLowMask)
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

    can2_rh = receive_handler;
    can2_sh = send_handler;
    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = ITP_CAN2_RX0_PREEMPTION;
    nvic.NVIC_IRQChannelSubPriority = ITP_CAN2_RX0_SUB;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = ITP_CAN2_TX_PREEMPTION;
    nvic.NVIC_IRQChannelSubPriority = ITP_CAN2_TX_SUB;
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
    can_filter.CAN_FilterScale = CAN_FilterScale_16bit;
    can_filter.CAN_FilterIdHigh = IdHigh << 5;
    can_filter.CAN_FilterIdLow = IdLow << 5;
    can_filter.CAN_FilterMaskIdHigh = IdHighMask << 5;
    can_filter.CAN_FilterMaskIdLow = IdLowMask << 5;
    can_filter.CAN_FilterFIFOAssignment = 1; //the message which pass the filter save in fifo0
    can_filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&can_filter);

    CAN_ITConfig(CAN2, CAN_IT_FMP1, ENABLE);
}

void CAN2_TX_IRQHandler(void)
{
    uint16_t id;
    int8_t code;
    if (CAN_GetITStatus(CAN2, CAN_IT_TME) != RESET)
    {
        if(can2_sh)
        {
            if(CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1))
            {
                if(CAN2->TSR & CAN_TSR_TME0)
                {
                    id = can2_mailbox0_id;
                    code = ((CAN2->TSR & CAN_TSR_TXOK0) == CAN_TSR_TXOK0);
                }
                else
                {
                    id = can2_mailbox1_id;
                    code = ((CAN2->TSR & CAN_TSR_TXOK1) == CAN_TSR_TXOK1);
                }
            }
            else
            {
                id = can2_mailbox2_id;
                code = ((CAN2->TSR & CAN_TSR_TXOK2) == CAN_TSR_TXOK2);
            }
        }
        CAN_ClearITPendingBit(CAN2, CAN_IT_TME);

        if(can2_sh)
            can2_sh(id, code);

        if(can2_mutex_transmit == 0 && can2_mutex_queue == 0 && (can2_pq_full == 1 || can2_pq_end != can2_pq_start))
        {
            can2_mutex_queue = 1;
            CAN2_Transmit(can2_pq[can2_pq_start].id, can2_pq[can2_pq_start].addr, can2_pq[can2_pq_start].data, can2_pq[can2_pq_start].size);
            can2_pq_full = 0;
            can2_pq_start = (can2_pq_start + 1) % CAN2_PACKET_QUEUE_SIZE;
            can2_mutex_queue = 0;
        }
    }
}

void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;

    if (CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx_message);
        if(can2_rh)
            can2_rh(&rx_message);
    }
}

int8_t CAN2_Transmit(uint16_t id, uint16_t addr, char* data, uint8_t size)
{
    CanTxMsg Tx_message;
    uint8_t used_mailbox;
    uint8_t i;
    int8_t ret;

    Tx_message.StdId = addr;
    Tx_message.IDE = CAN_Id_Standard;
    Tx_message.RTR = CAN_RTR_Data;
    Tx_message.DLC = size;
    for (i = 0; i < size; i++)
        Tx_message.Data[i] = data[i] & 0xff;

    if(can2_mutex_transmit == 0) {
        can2_mutex_transmit = 1;
        used_mailbox = CAN_Transmit(CAN2, &Tx_message);
        if (used_mailbox != CAN_TxStatus_NoMailBox)
        {
            switch (used_mailbox) {
                case 0:
                    can2_mailbox0_id = id;
                    break;
                case 1:
                    can2_mailbox1_id = id;
                    break;
                case 2:
                    can2_mailbox2_id = id;
                    break;
            }
            ret = 1;
        }
        else
        {
            if (can2_sh)
                can2_sh(id, 0);
            printf("[CAN2_Transmit] No mailbox\r\n");
            ret = -1;
        }

        can2_mutex_transmit = 0;
    }
    else
        ret = -1;
    return ret;
}

int8_t CAN2_AsyncTransmit(uint16_t id, uint16_t addr, char* data, uint8_t size)
{
    int8_t ret;
    if((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2))
        && CAN2_Transmit(id, addr, data, size) == 1)
    {
        return 1;
    }

    if(can2_mutex_queue == 0) {
        can2_mutex_queue = 1;
        can2_async_transmit_times++;
        if (can2_pq_full == 0) {
            can2_pq[can2_pq_end].id = id;
            can2_pq[can2_pq_end].addr = addr;
            can2_pq[can2_pq_end].size = size;
            can2_pq[can2_pq_end].data[0] = data[0];
            can2_pq[can2_pq_end].data[1] = data[1];
            can2_pq[can2_pq_end].data[2] = data[2];
            can2_pq[can2_pq_end].data[3] = data[3];
            can2_pq[can2_pq_end].data[4] = data[4];
            can2_pq[can2_pq_end].data[5] = data[5];
            can2_pq[can2_pq_end].data[6] = data[6];
            can2_pq[can2_pq_end].data[7] = data[7];
            can2_pq_end = (can2_pq_end + 1) % CAN2_PACKET_QUEUE_SIZE;
            if (can2_pq_start == can2_pq_end) can2_pq_full = 1;
            ret = 0;
        }
        else {
            can2_queue_full_times++;
            if (can2_sh)
                can2_sh(id, 0);
            printf("[CAN2_AsyncTransmit] Queue Full\r\n");
            ret = -1;
        }
        can2_mutex_queue = 0;
    }
    else
        ret = -1;
    return ret;

}