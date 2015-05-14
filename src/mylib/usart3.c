#include <stdio.h>
#include "stm32f4xx.h"
#include "receiver.h"
#include "led.h"
#include "stm32f4xx_it.h"
#include "usart3.h"

/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/

void USART3_Configuration(void)
{
	USART_InitTypeDef usart3;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef dma;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpio);

	usart3.USART_BaudRate = 115200;
	usart3.USART_WordLength = USART_WordLength_8b;
	usart3.USART_StopBits = USART_StopBits_1;
	usart3.USART_Parity = USART_Parity_No;
	usart3.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_DeInit(USART3);
	USART_Init(USART3, &usart3);

	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = ITP_USART3_GLOBAL_PREEMPTION;
	nvic.NVIC_IRQChannelSubPriority = ITP_USART3_GLOBAL_SUB;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	// USART3_RX
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	dma.DMA_Memory0BaseAddr = 0;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = 0;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Cmd(DMA1_Stream1, DISABLE);
	DMA_DeInit(DMA1_Stream1);
	DMA_Init(DMA1_Stream1, &dma);


	nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = ITP_USART3_DMA_RX_PREEMPTION;
	nvic.NVIC_IRQChannelSubPriority = ITP_USART3_DMA_RX_SUB;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);



	// USART3_TX
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	dma.DMA_Memory0BaseAddr = 0;
	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma.DMA_BufferSize = 0;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_DeInit(DMA1_Stream3);
	DMA_Init(DMA1_Stream3, &dma);

	nvic.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = ITP_USART3_DMA_TX_PREEMPTION;
	nvic.NVIC_IRQChannelSubPriority = ITP_USART3_DMA_TX_SUB;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	DMA_ITConfig(DMA1_Stream1, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC | DMA_IT_TE, ENABLE);

	//USART_DMACmd(USART3, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

	USART_Cmd(USART3, ENABLE);

}

void (*rx_handler)(char) = NULL;

void USART3_Set_Rx_Handler(void (*rxh)(char))
{
	rx_handler = rxh;
}

void USART3_IRQHandler(void)
{

	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		//USART_ClearFlag(USART3, USART_FLAG_RXNE);
		if(rx_handler) rx_handler(USART3->DR);
	}
}

void (*dma_tx_cb_func)(int8_t) = NULL;

int USART3_DMA_SendData(char *data, uint8_t size, void (*callback_func)(int8_t))
{
	if(DMA1_Stream3->CR & DMA_SxCR_EN)
	{
		return -1;
	}

	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TCIF3);

	DMA_SetCurrDataCounter(DMA1_Stream3, size);
	DMA1_Stream3->M0AR = (uint32_t)data;
	dma_tx_cb_func = callback_func;

	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Stream3, ENABLE);


	return size;
}

void DMA1_Stream3_IRQHandler(void)
{
	int result = 0;
	USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);

	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) == SET)
		result = 1;

	DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3 | DMA_IT_TEIF3);
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TCIF3);

	DMA_Cmd(DMA1_Stream3, DISABLE);

	if(dma_tx_cb_func)
		dma_tx_cb_func(result);

}

void (*dma_rx_cb_func)(int8_t) = NULL;

int USART3_DMA_ReceiveData(char* data, uint8_t size, void (*callback_func)(int8_t))
{

	if(DMA1_Stream1->CR & DMA_SxCR_EN)
	{
		return -1;
	}
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);

	DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TCIF1);

	DMA_SetCurrDataCounter(DMA1_Stream1, size);
	DMA1_Stream1->M0AR = (uint32_t)data;
	dma_rx_cb_func = callback_func;

	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Stream1, ENABLE);

	return size;
}
// USART3 DMA RX
void DMA1_Stream1_IRQHandler(void)
{
	int result = 0;
	USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);

	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1) == SET)
		result = 1;
	DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1 | DMA_IT_TEIF1);
	DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TCIF1);

	DMA_Cmd(DMA1_Stream1, DISABLE);
	if(dma_rx_cb_func)
		dma_rx_cb_func(result);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

}

void USART3_SendChar(unsigned char b)
{
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		;
	USART_SendData(USART3, b);
}

int USART3_SendStr(char* ptr, int len)
{
	int i = 0;
	for (i = 0; i< len; i++)
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
			;
		USART_SendData(USART3, ptr[i]);
	}
	return i;
}

#if defined STDOUT_USART3

#if defined ( __CC_ARM )

int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		;
	USART_SendData(USART3, (uint8_t) ch);
	return ch;
}

#elif defined ( __GNUC__ )

int _write(int file, char* ptr, int len)
{
	int i = 0;
	for (i = 0; i< len; i++)
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
			;
		USART_SendData(USART3, ptr[i]);
	}
	return i;
}

#endif /*__CC_ARM OR __GNUC__*/

#endif /*defined STDOUT_USART3*/

