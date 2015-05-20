#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "receiver.h"
#include "ticker.h"

/*-----USART2_RX-----PA3----*/ 
//for D-BUS

uint16_t movespeed_1 = 1024;
uint16_t movespeed_2 = 0;
float pitch_propotion;
int16_t Gim_yaw_temp;

unsigned char sbus_rx_buffer[18];

Receiver_Packet receiver_packet;
int8_t mutex_receiving_packet = 0;
int8_t mutex_read_packet = 0;
int8_t new_receiver_packet = 0;

uint32_t resync_time_span = 0;
uint64_t resync_last_rec = 0;

#define SWITCH_DMA_DIRECT_DIRECT 0
#define SWITCH_DMA_DIRECT_DMA 1

void Switch_DMA_Direct(uint8_t option)
{
    if(option == SWITCH_DMA_DIRECT_DMA)
    {
        USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
        USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
        DMA_Cmd(DMA1_Stream5, ENABLE);
    }
    else
    {
        DMA_Cmd(DMA1_Stream5,DISABLE);
        USART_DMACmd(USART2, USART_DMAReq_Rx, DISABLE);
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    }
}


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


		nvic.NVIC_IRQChannel = DMA1_Stream5_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = ITP_USART2_DMA_RX_PREEMPTION;
		nvic.NVIC_IRQChannelSubPriority = ITP_USART2_DMA_RX_SUB;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);

        nvic.NVIC_IRQChannel = USART2_IRQn;
        nvic.NVIC_IRQChannelPreemptionPriority = ITP_USART2_PREEMPTION;
        nvic.NVIC_IRQChannelSubPriority = ITP_USART2_SUB;
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
		dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream5,&dma);

		DMA_ITConfig(DMA1_Stream5,DMA_IT_TC | DMA_IT_TE | DMA_IT_FE,ENABLE);

        resync_time_span = Ticker_Get_MS_Tickcount() * 5;
        Switch_DMA_Direct(SWITCH_DMA_DIRECT_DMA);
}

int8_t Receiver_Get_New_Packet(Receiver_Packet *rp)
{
    int8_t ret = 0;
    mutex_read_packet = 1;
    if(mutex_receiving_packet == 0) {
        if (new_receiver_packet == 1) {
            memcpy(rp, &receiver_packet, sizeof(receiver_packet));
            new_receiver_packet = 0;
            ret = 1;
        }
    }
    mutex_read_packet = 0;
    return ret;
}


void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        if(Ticker_Get_Tick() - resync_last_rec > resync_time_span)
        {
            Switch_DMA_Direct(SWITCH_DMA_DIRECT_DMA);
            return;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        resync_last_rec = Ticker_Get_Tick();
    }
}




uint16_t  sbus_channel_temp[15] = {0};  //  temp sbus decode channel data
uint16_t  radio_yuntai_temp[4] = {0};  //

uint16_t  Move_Speed_X=1024, Move_Speed_Y=1024,Rotate=1024;
uint16_t  Move_Straight,Move_Horizontal;


void DMA1_Stream5_IRQHandler(void)
{
	uint8_t fault_detect = 0;
	
	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5)) {
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);

        mutex_receiving_packet = 1;
        if(mutex_read_packet == 0) {
            receiver_packet.ch0 =
                    (uint16_t) ((sbus_rx_buffer[0] | (sbus_rx_buffer[1] << 8)) & 0x07ff); // right_horizontal 364-1024-1684;
            receiver_packet.ch1 = (uint16_t) (((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff); // right_vertical
            receiver_packet.ch2 = (uint16_t) (((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff); // left_horizontal
            receiver_packet.ch3 = (uint16_t) (((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff); // left_vertical
            // disagree with protocol described in 遥控器控制协议v1.4.pdf
            // [44:45] right, [46:47] left
            receiver_packet.s1 = (uint8_t) ((sbus_rx_buffer[5] >> 6) & 0x3); // radio_switch_left 1 2 3
            receiver_packet.s2 = (uint8_t) ((sbus_rx_buffer[5] >> 4) & 0x3); // radio_switch_right
            receiver_packet.mouse_x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); // -32768-0-32767
            receiver_packet.mouse_y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);
            receiver_packet.mouse_z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);
            receiver_packet.mouse_left = sbus_rx_buffer[12];// 0 1
            receiver_packet.mouse_right = sbus_rx_buffer[13];//
            receiver_packet.key_pressd = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);
            receiver_packet.reserved = sbus_rx_buffer[16] | (sbus_rx_buffer[17] << 8);

            // Fault Detect
            if (receiver_packet.ch0 < 364 || receiver_packet.ch0 > 1684 ||
                                             receiver_packet.ch1 < 364 || receiver_packet.ch1 > 1684 ||
                                             receiver_packet.ch2 < 364 || receiver_packet.ch2 > 1684 ||
                                             receiver_packet.ch3 < 364 || receiver_packet.ch3 > 1684) {
                fault_detect = 1;
            }

            if (receiver_packet.s1 < 1 || receiver_packet.s1 > 3 || receiver_packet.s2 < 1 || receiver_packet.s2 > 3) {
                fault_detect = 1;
            }

            // assume to be 0
            if (receiver_packet.key_pressd & 0xff00) {
                fault_detect = 1;
            }

            if (receiver_packet.reserved != 0) {
                fault_detect = 1;
            }

            if (fault_detect) {
                // Resync
                resync_last_rec = Ticker_Get_Tick();
                Switch_DMA_Direct(SWITCH_DMA_DIRECT_DIRECT);
            }
            else
            {
                new_receiver_packet = 1;
            }
        }
        mutex_receiving_packet = 0;
    }
}

