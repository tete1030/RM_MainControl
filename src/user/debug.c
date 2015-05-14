#include <stdio.h>
#include <stdint.h>
#include "debug.h"
#include "usart3.h"
#include "driver.h"


char COMPUTER_DATA_START[] = { 0x12, 0x23, 0x34, 0x45 };
#define COMPUTER_DATA_START_LEN (sizeof(COMPUTER_DATA_START))

#define COMPUTER_DATA_ACK_ACCEPT 0x69
#define COMPUTER_DATA_ACK_REJECT 0x96
#define COMPUTER_DATA_RECEIVE_FAILED 0xbb
#define COMPUTER_DATA_CHK_SUCCESS 0xcc
#define COMPUTER_DATA_CHK_FAILED 0xdd

#define RECEIVE_STATE_IDLE 0
#define RECEIVE_STATE_START 1
#define RECEIVE_STATE_CHECK 2
#define RECEIVE_STATE_RXFAILED 3
#define RECEIVE_STATE_RECEIVING 4

int8_t receive_state = 0;
int8_t receive_new_data = 0;

#define MAX_RECEIVE_DATA_SIZE 32

char receive_data[MAX_RECEIVE_DATA_SIZE];
uint8_t receive_data_size = 0;

void Debug_DMARx_Callback(int8_t f)
{
	if(f == 0)
	{
		receive_state = RECEIVE_STATE_RXFAILED;
	}
	else
	{
		receive_state = RECEIVE_STATE_CHECK;
	}


}

void Debug_Rx_Callback(char c)
{
	uint8_t cur_chksum;
	uint8_t i;
	static int8_t start_counter = 0;
	if(receive_state == RECEIVE_STATE_IDLE)
	{
		if(c == COMPUTER_DATA_START[start_counter])
		{
			start_counter ++;
			if(start_counter == COMPUTER_DATA_START_LEN)
			{
				receive_state = RECEIVE_STATE_START;
			}
		}
		else start_counter = 0;
	}
	else if(receive_state == RECEIVE_STATE_START)
	{
		receive_data_size = (uint8_t)c;
		if(receive_data_size <= MAX_RECEIVE_DATA_SIZE)
		{
			USART3_DMA_ReceiveData(receive_data, receive_data_size, Debug_DMARx_Callback);
			USART3_SendChar(COMPUTER_DATA_ACK_ACCEPT);
			receive_state = RECEIVE_STATE_RECEIVING;
		}
		else
		{
			USART3_SendChar(COMPUTER_DATA_ACK_REJECT);
			receive_state = RECEIVE_STATE_IDLE;
		}
	}
	else if(receive_state == RECEIVE_STATE_RECEIVING)
	{
		// Fault!!!!!!!!!
	}
	else if(receive_state == RECEIVE_STATE_RXFAILED)
	{
		USART3_SendChar(COMPUTER_DATA_RECEIVE_FAILED);
		receive_state = RECEIVE_STATE_IDLE;
	}
	else if(receive_state == RECEIVE_STATE_CHECK)
	{
		cur_chksum = 0;
		for(i=0; i<receive_data_size; i++)
		{
			cur_chksum += receive_data[i];
		}

		if(cur_chksum == (uint8_t)c)
		{
			receive_new_data = 1;
			USART3_SendChar(COMPUTER_DATA_CHK_SUCCESS);
		}
		else
		{
			receive_new_data = 0;
			USART3_SendChar(COMPUTER_DATA_CHK_FAILED);
		}
		receive_state = RECEIVE_STATE_IDLE;
	}
	else
		receive_state = RECEIVE_STATE_IDLE;
}


char MAINCONTROL_DATA_START[] = { 0x13, 0x24, 0x35, 0x46 };
#define MAINCONTROL_DATA_START_LEN (sizeof(MAINCONTROL_DATA_START))

#define TRANSMIT_DATA_SIZE_HEADER 8
#define TRANSMIT_DATA_OFFSET_SOURCE 4
#define TRANSMIT_DATA_OFFSET_TYPE 5
#define TRANSMIT_DATA_OFFSET_ID 6
#define TRANSMIT_DATA_OFFSET_SIZE 7

#define TRANSMIT_DATA_SIZE_BODY_MAX 64
#define TRANSMIT_DATA_OFFSET_BODY TRANSMIT_DATA_SIZE_HEADER

char transmit_data[TRANSMIT_DATA_SIZE_BODY_MAX+TRANSMIT_DATA_SIZE_HEADER];
int8_t last_transmit_complete = 1;
uint8_t transmit_counter = 0;

void Debug_DMATx_Callback(int8_t f)
{
	last_transmit_complete = 1;
}

int Debug_Transmit(int8_t source, int8_t type, char* data, uint8_t size)
{
	int ret;
	uint8_t checksum = 0;
	if((size) > TRANSMIT_DATA_SIZE_BODY_MAX) return -1;
	if(last_transmit_complete == 0) return -2;
	last_transmit_complete = 0;
	transmit_data[TRANSMIT_DATA_OFFSET_SOURCE] = source;
	transmit_data[TRANSMIT_DATA_OFFSET_TYPE] = type;
	transmit_data[TRANSMIT_DATA_OFFSET_ID] = ++transmit_counter;
	transmit_data[TRANSMIT_DATA_OFFSET_SIZE] = size;
	for(; size > 0; size--)
	{
		checksum += (transmit_data[TRANSMIT_DATA_OFFSET_BODY + size - 1] = data[size-1]);
	}
	transmit_data[TRANSMIT_DATA_OFFSET_BODY + size] = checksum;
	ret = USART3_DMA_SendData(transmit_data, TRANSMIT_DATA_SIZE_HEADER + size + 1, Debug_DMATx_Callback);
	return ret;
}

int Debug_Transmit_Filled_Data(int8_t source, int8_t type, uint8_t size)
{
	int ret;
	uint8_t checksum = 0;
	if((size) > TRANSMIT_DATA_SIZE_BODY_MAX) return -1;
	if(last_transmit_complete == 0) return -2;
	last_transmit_complete = 0;
	transmit_data[TRANSMIT_DATA_OFFSET_SOURCE] = source;
	transmit_data[TRANSMIT_DATA_OFFSET_TYPE] = type;
	transmit_data[TRANSMIT_DATA_OFFSET_ID] = ++transmit_counter;
	transmit_data[TRANSMIT_DATA_OFFSET_SIZE] = size;
	for(; size > 0; size--)
	{
		checksum += transmit_data[TRANSMIT_DATA_OFFSET_BODY + size - 1];
	}
	transmit_data[TRANSMIT_DATA_OFFSET_BODY + size] = checksum;
	ret = USART3_DMA_SendData(transmit_data, TRANSMIT_DATA_SIZE_HEADER + size + 1, Debug_DMATx_Callback);
	return ret;
}

#if defined STDOUT_DEBUG

#if defined ( __CC_ARM )

#include <stdarg.h>


uint8_t send_buffer_count = 0;

int fputc(int ch, FILE *f)
{
	transmit_data[TRANSMIT_DATA_OFFSET_BODY + send_buffer_count++] = (uint8_t) ch;
	return ch;
}

int printf(const char * __restrict format, ...)
{
	va_list vargs;
	int result;
	va_start(vargs, format);
	result = vfprintf(stdout, format, vargs);
	va_end(vargs);

	send_buffer_count = 0;
	if(result > 0)
		Debug_Transmit_Filled_Data(MAINCONTROL_DATA_SOURCE_MAINCONTROL, MAINCONTROL_TYPE_PRINTF, result);

	return result;
}

#elif defined ( __GNUC__ )

int _write(int file, char* ptr, int len)
{
	int i;
	Debug_Transmit(MAINCONTROL_DATA_SOURCE_MAINCONTROL, MAINCONTROL_TYPE_PRINTF, ptr, len);
	return i;
}

#endif /*__CC_ARM OR __GNUC__*/

#endif /*defined STDOUT_DEBUG*/

void Debug_Configuration()
{
	uint8_t i;
	USART3_Set_Rx_Handler(Debug_Rx_Callback);
	for(i = 0; i < MAINCONTROL_DATA_START_LEN; i++)
	{
		transmit_data[i] = MAINCONTROL_DATA_START[i];
	}
}

#define COMPUTER_COMMAND_ENABLE 0x32
#define COMPUTER_COMMAND_CONFIG 0x33
#define COMPUTER_COMMAND_SPEED 0x34
#define COMPUTER_COMMAND_WHEELSPEED 0x35

void Debug_Execute_Computer_Command()
{
	if(receive_new_data == 1)
	{
		switch(receive_data[0])
		{
		case COMPUTER_COMMAND_ENABLE:
			Driver_Set_Enable(receive_data[1], receive_data[2]);
			break;
		case COMPUTER_COMMAND_CONFIG:
			Driver_Set_Configuration(receive_data[1], (Motor_Config_Type)(receive_data[2]), (void*)(receive_data + 3));
			break;
		case COMPUTER_COMMAND_SPEED:
			Driver_Set_Speed(((int16_t*)(receive_data+1))[0], ((int16_t*)(receive_data+1))[1], ((float*)(((int16_t*)(receive_data+1))+2))[0]);
			break;
		case COMPUTER_COMMAND_WHEELSPEED:
			Driver_Set_Wheel_Speed(receive_data[1], ((int16_t*)(receive_data+2))[0]);
			break;
		}
		receive_new_data = 0;
	}
}
