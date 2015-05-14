#include <stdio.h>
#include "stm32f4xx.h"
#include "can1.h"
#include "driver.h"
#include "main.h"
#include "debug.h"

/* aaa bbbc dddd

 * aaa:  目标
 * bbb:  数据包类型
 * c:    数据放置类型
 * dddd: 目标地址
 */
#define CAN_PACKET_TYPE_MASK_DESTTYPE 0x700				/* 111 0000	0000 */
#define CAN_PACKET_TYPE_MASK_DATATYPE 0x0e0				/* 000 1110	0000 */
#define CAN_PACKET_TYPE_MASK_PLACETYPE 0x010			/* 000 0001	0000 */
#define CAN_PACKET_TYPE_MASK_DESTADDR_STRICT 0xf
#define CAN_PACKET_TYPE_MASK_DESTADDR_ALL 0

#define CAN_PACKET_DESTTYPE_DRIVER \
	(0x0 << 8)									/* 000 xxxx xxxx */
#define CAN_PACKET_DESTTYPE_CENTER \
	(0x1 << 8)									/* 001 xxxx xxxx */


#define CAN_PACKET_DRIVER_DATATYPE_ENABLE \
	(0x0 << 5)									/* xxx 000x xxxx */
#define CAN_PACKET_DRIVER_DATATYPE_CONFIG \
	(0x1 << 5)									/* xxx 001x xxxx */
#define CAN_PACKET_DRIVER_DATATYPE_CONTROL \
	(0x2 << 5)									/* xxx 002x xxxx */


#define CAN_PACKET_CENTER_DATATYPE_PRINTF \
	(0x0 << 5)									/* xxx 000x xxxx */
#define CAN_PACKET_CENTER_DATATYPE_STATUS \
	(0x1 << 5)									/* xxx 000x xxxx */


#define CAN_PACKET_PLACETYPE_ALLINONE \
	(0x0 << 4)									/* xxx xxx0 xxxx */
#define CAN_PACKET_PLACETYPE_SPECIFIC \
	(0x1 << 4)									/* xxx xxx1 xxxx */


#define DRIVER_ENCODER_SAMPLE_DURATION 5
#define DRIVER_ENCODER_SPEED_FACTOR (1000/DRIVER_ENCODER_SAMPLE_DURATION)

#define CAR_WIDTH_PLUS_LENGTH_N_DIV_2 ((CAR_WIDTH + CAR_LENGTH) / 2)

int8_t Can1_Tx_Buffer[8];

void Driver_Configuration()
{
	extern void Driver_Can_Receive_Handler(CanRxMsg* rx_msg);
	extern void Driver_Can_Send_Handler(uint16_t id, int8_t code);
	CAN1_Configuration(Driver_Can_Send_Handler, Driver_Can_Receive_Handler,
			CAN_PACKET_DESTTYPE_CENTER | CAN_PACKET_PLACETYPE_ALLINONE | CAN_DRIVER_SELECT_ALL_ADDR,
			CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_DESTADDR_ALL,
			CAN_PACKET_DESTTYPE_CENTER | CAN_PACKET_PLACETYPE_SPECIFIC | CAN_DRIVER_SELECT_ALL_ADDR,
			CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_DESTADDR_ALL);

}

#define DRIVER_PACKET_ID_ENABLE 0x1

void Driver_Set_Enable(uint8_t motor_sel, uint8_t motor_enable_sel)
{
	Can1_Tx_Buffer[0] = motor_enable_sel;
	CAN1_AsyncTransmit(DRIVER_PACKET_ID_ENABLE,
			CAN_PACKET_DESTTYPE_DRIVER |
			CAN_PACKET_DRIVER_DATATYPE_ENABLE |
			CAN_PACKET_PLACETYPE_ALLINONE |
			motor_sel,
			Can1_Tx_Buffer, 1);
}

#define DRIVER_PACKET_ID_SPEED 0x2

// Set Car Speed
// Unit: vx, vy is mm/s, w0 is rad/s
void Driver_Set_Speed(int16_t vx, int16_t vy, float w0)
{
	int16_t wlf, wrf, wle, wre;

	int32_t p = (int32_t) (vx + vy) * CAR_ENCODER_NUM;
	int32_t q = (int32_t) (vx - vy) * CAR_ENCODER_NUM;
	float m = w0 * CAR_WIDTH_PLUS_LENGTH_N_DIV_2 * CAR_ENCODER_NUM;
	float n = 2 * PI * CAR_WHEEL_RADIUS * DRIVER_ENCODER_SPEED_FACTOR;

	wlf = ((float) p - m) / n;
	wrf = ((float) q + m) / n;
	wle = ((float) q - m) / n;
	wre = ((float) p + m) / n;

	//printf("%d, %d, %d, %d\r\n", wlf, wrf, wle, wre);

	((int16_t*) Can1_Tx_Buffer)[0] = wlf;
	((int16_t*) Can1_Tx_Buffer)[1] = wrf;
	((int16_t*) Can1_Tx_Buffer)[2] = wle;
	((int16_t*) Can1_Tx_Buffer)[3] = wre;

	CAN1_AsyncTransmit(DRIVER_PACKET_ID_SPEED,
			CAN_PACKET_DESTTYPE_DRIVER |
			CAN_PACKET_DRIVER_DATATYPE_CONTROL |
			CAN_PACKET_PLACETYPE_ALLINONE |
			CAN_DRIVER_LEFT_FRONT_ADDR |
			CAN_DRIVER_RIGHT_FRONT_ADDR |
			CAN_DRIVER_LEFT_END_ADDR |
			CAN_DRIVER_RIGHT_END_ADDR,
			Can1_Tx_Buffer, 8);
}

#define DRIVER_PACKET_ID_WHEEL_SPEED 0x3

void Driver_Set_Wheel_Speed(uint8_t motor_sel, int16_t destSpeed)
{
	Can1_Tx_Buffer[0] = destSpeed & 0xff;
	Can1_Tx_Buffer[1] = (destSpeed >> 8) & 0xff;
	CAN1_AsyncTransmit(DRIVER_PACKET_ID_WHEEL_SPEED,
			CAN_PACKET_DESTTYPE_DRIVER |
			CAN_PACKET_DRIVER_DATATYPE_CONTROL |
			CAN_PACKET_PLACETYPE_SPECIFIC |
			motor_sel,
			Can1_Tx_Buffer, 2);
}

#define DRIVER_PACKET_ID_CONFIG 0x4

void Driver_Set_Configuration(uint8_t motor_sel, Motor_Config_Type mct, void* value)
{
	uint8_t send_size = 0;
	uint8_t send_addr = 0;
	Can1_Tx_Buffer[0] = (int8_t)mct;
	switch(mct)
	{
	case MCT_Max_Speed:
		((uint16_t*)(Can1_Tx_Buffer+1))[0] = *((uint16_t*)value);
		send_size = 3;
		break;
	case MCT_PID:
		Can1_Tx_Buffer[1] = ((int8_t*)value)[0];
		Can1_Tx_Buffer[2] = ((int8_t*)value)[1];
		Can1_Tx_Buffer[3] = ((int8_t*)value)[2];
		send_size = 4;
		break;
	case MCT_Speed_Step:
		((uint16_t*)(Can1_Tx_Buffer+1))[0] = *((uint16_t*)value);
		send_size = 3;
		break;
	}

	if(send_size > 0)
	{
		CAN1_AsyncTransmit(DRIVER_PACKET_ID_CONFIG,
				CAN_PACKET_DESTTYPE_DRIVER |
				CAN_PACKET_DRIVER_DATATYPE_CONFIG |
				CAN_PACKET_PLACETYPE_SPECIFIC |
				motor_sel,
				Can1_Tx_Buffer, send_size);
	}

}



void Driver_Can_Receive_Handler(CanRxMsg* rx_msg)
{
	int16_t type,id;
	int8_t i;
	type = rx_msg->StdId & CAN_PACKET_TYPE_MASK_DATATYPE;
	id = rx_msg->StdId & CAN_PACKET_TYPE_MASK_DESTADDR_STRICT;
	if(type == CAN_PACKET_CENTER_DATATYPE_PRINTF)
	{

	}
	else if(type == CAN_PACKET_CENTER_DATATYPE_STATUS)
	{

	}

}


void Driver_Can_Send_Handler(uint16_t id, int8_t code)
{
	if(code == 0)printf("%u send failed\r\n", id);
}
