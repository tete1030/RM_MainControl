#ifndef __USART3_H__
#define __USART3_H__


void USART3_Configuration(void);
void USART3_SendChar(unsigned char b);
int USART3_DMA_SendData(char *data, uint8_t size, void (*callback_func)(int8_t));
int USART3_DMA_ReceiveData(char* data, uint8_t size, void (*callback_func)(int8_t));
void USART3_Set_Rx_Handler(void (*rxh)(char));

#endif
