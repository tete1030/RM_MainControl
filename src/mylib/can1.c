//
// Created by Texot Qexoq on 5/16/15.
//
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "can1.h"
#include "can_packet_queue.h"

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/
void (*can1_rh)(CanRxMsg*);
void (*can1_sh)(uint16_t, int8_t);

#define CAN1_PACKET_QUEUE_SIZE 10

volatile uint16_t can1_mailbox0_id =0;
volatile uint16_t can1_mailbox1_id =0;
volatile uint16_t can1_mailbox2_id =0;

volatile CAN_Packet_Queue can1_pq[CAN1_PACKET_QUEUE_SIZE];
volatile uint8_t can1_pq_start = 0;
volatile uint8_t can1_pq_end = 0;
volatile int8_t can1_pq_full = 0;

volatile uint32_t can1_async_transmit_times = 0;
volatile uint32_t can1_queue_full_times = 0;

volatile int8_t can1_mutex_transmit = 0;
volatile int8_t can1_mutex_queue = 0;

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

    can1_rh = receive_handler;
    can1_sh = send_handler;
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

    can.CAN_Prescaler = 3;
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
    int16_t id;
    int8_t code;
    uint32_t tsr;
    if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
    {
        tsr = CAN1->TSR;
        if(can1_sh)
        {
            if((tsr & CAN_TSR_TME0) && (tsr & CAN_TSR_RQCP0))
            {
                CAN1->TSR = CAN_TSR_RQCP0;
                id = can1_mailbox0_id;
                if (tsr & CAN_TSR_TXOK0)
                    code = 0;
                else if (tsr & CAN_TSR_TERR0)
                    code = 1;
                else if (tsr & CAN_TSR_ALST0)
                    code = 3;
                else
                    code = 4;
            }
            else if((tsr & CAN_TSR_TME1) && (tsr & CAN_TSR_RQCP1))
            {
                CAN1->TSR = CAN_TSR_RQCP1;
                id = can1_mailbox1_id;
                if (tsr & CAN_TSR_TXOK1)
                    code = 0;
                else if (tsr & CAN_TSR_TERR1)
                    code = 1;
                else if (tsr & CAN_TSR_ALST1)
                    code = 3;
                else
                    code = 4;
            }
            else if((tsr & CAN_TSR_TME2) && (tsr & CAN_TSR_RQCP2))
            {
                CAN1->TSR = CAN_TSR_RQCP2;
                id = can1_mailbox2_id;
                if (tsr & CAN_TSR_TXOK2)
                    code = 0;
                else if (tsr & CAN_TSR_TERR2)
                    code = 1;
                else if (tsr & CAN_TSR_ALST2)
                    code = 3;
                else
                    code = 4;
            }
            else
            {
                id = -1;
            }

            if(id >= 0)
                can1_sh(id, code);
        }

        if(can1_mutex_transmit == 0 && can1_mutex_queue == 0 && (can1_pq_full == 1 || can1_pq_end != can1_pq_start))
        {
            can1_mutex_queue = 1;
            CAN1_Transmit(can1_pq[can1_pq_start].id, can1_pq[can1_pq_start].addr, can1_pq[can1_pq_start].data, can1_pq[can1_pq_start].size);
            can1_pq_full = 0;
            can1_pq_start = (can1_pq_start + 1) % CAN1_PACKET_QUEUE_SIZE;
            can1_mutex_queue = 0;
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
        if(can1_rh)
            can1_rh(&rx_message);
    }
}

int8_t CAN1_Transmit(uint16_t id, uint16_t addr, char* data, uint8_t size)
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
        Tx_message.Data[i] = (uint8_t) (data[i] & 0xff);

    if(can1_mutex_transmit == 0) {
        can1_mutex_transmit = 1;
        used_mailbox = CAN_Transmit(CAN1, &Tx_message);

        if (used_mailbox != CAN_TxStatus_NoMailBox) {
            switch (used_mailbox) {
                case 0:
                    can1_mailbox0_id = id;
                    break;
                case 1:
                    can1_mailbox1_id = id;
                    break;
                case 2:
                    can1_mailbox2_id = id;
                    break;
            }
            ret = 1;
        }
        else {
            if (can1_sh)
                can1_sh(id, 0);
            printf("[CAN1_Transmit] No mailbox\r\n");
            ret = -1;
        }
        can1_mutex_transmit = 0;
    }
    else
        ret = -1;
    return ret;
}

int8_t CAN1_AsyncTransmit(uint16_t id, uint16_t addr, char* data, uint8_t size)
{
    int8_t ret;
    if((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2))
       && (CAN1_Transmit(id, addr, data, size) == 1))
    {
        return 1;
    }

    if(can1_mutex_queue == 0) {
        can1_mutex_queue = 1;
        can1_async_transmit_times++;
        if (can1_pq_full == 0) {
            can1_pq[can1_pq_end].id = id;
            can1_pq[can1_pq_end].addr = addr;
            can1_pq[can1_pq_end].size = size;
            can1_pq[can1_pq_end].data[0] = data[0];
            can1_pq[can1_pq_end].data[1] = data[1];
            can1_pq[can1_pq_end].data[2] = data[2];
            can1_pq[can1_pq_end].data[3] = data[3];
            can1_pq[can1_pq_end].data[4] = data[4];
            can1_pq[can1_pq_end].data[5] = data[5];
            can1_pq[can1_pq_end].data[6] = data[6];
            can1_pq[can1_pq_end].data[7] = data[7];
            can1_pq_end = (can1_pq_end + 1) % CAN1_PACKET_QUEUE_SIZE;
            if (can1_pq_start == can1_pq_end) can1_pq_full = 1;
            ret = 0;
        }
        else {
            can1_queue_full_times++;
            if (can1_sh)
                can1_sh(id, 0);
            printf("[CAN1_AsyncTransmit] Queue Full\r\n");
            ret = -1;
        }
        can1_mutex_queue = 0;
    }
    else
        ret = -1;
    return ret;

}