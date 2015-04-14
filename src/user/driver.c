#include <stdio.h>
#include "stm32f4xx.h"
#include "can1.h"
#include "main.h"

#define CAN_ENABLE_TYPE 0x65
#define CAN_CONTROL_TYPE 0x66
#define CAN_DATA_TYPE 0x67

#define CAN_DRIVER_LEFT_FRONT_ADDR 0x1
#define CAN_DRIVER_RIGHT_FRONT_ADDR 0x2
#define CAN_DRIVER_LEFT_END_ADDR 0x3
#define CAN_DRIVER_RIGHT_END_ADDR 0x4

#define CAR_WIDTH_PLUS_LENGTH_N_DIV_2 ((CAR_WIDTH + CAR_LENGTH) / 2)

char Can1_Tx_Buffer[8];

void Driver_Configuration()
{
	extern void Driver_Data_Handler(CanRxMsg* rx_msg);
	CAN1_Configuration(Driver_Data_Handler);
}

void Driver_Enable(uint8_t motor_en)
{
	Can1_Tx_Buffer[0] = motor_en & 1;
	Can1_Tx_Buffer[1] = 0;
	Can1_Tx_Buffer[2] = (motor_en & 2) >> 1;
	Can1_Tx_Buffer[3] = 0;
	Can1_Tx_Buffer[4] = (motor_en & 4) >> 2;
	Can1_Tx_Buffer[5] = 0;
	Can1_Tx_Buffer[6] = (motor_en & 8) >> 3;
	Can1_Tx_Buffer[7] = 0;
	CAN1_Transmit(CAN_ENABLE_TYPE << 3, Can1_Tx_Buffer, 8);
}

// Set Car Speed
// Unit: vx, vy is mm/s, w0 is rad/s
void Driver_Set_Speed(int16_t vx, int16_t vy, float w0)
{
	int16_t wlf, wrf, wle, wre;

	int32_t p = (int32_t) (vx + vy) * CAR_ENCODER_NUM;
	int32_t q = (int32_t) (vx - vy) * CAR_ENCODER_NUM;
	float m = w0 * CAR_WIDTH_PLUS_LENGTH_N_DIV_2 * CAR_ENCODER_NUM;
	float n = 2 * PI * CAR_WHEEL_RADIUS;

	wlf = ((float) p - m) / n;
	wrf = ((float) q + m) / n;
	wle = ((float) q - m) / n;
	wre = ((float) p + m) / n;

	printf("%d, %d, %d, %d\r\n", wlf, wrf, wle, wre);

	((int16_t*) Can1_Tx_Buffer)[0] = wlf;
	((int16_t*) Can1_Tx_Buffer)[1] = wrf;
	((int16_t*) Can1_Tx_Buffer)[2] = wle;
	((int16_t*) Can1_Tx_Buffer)[3] = wre;
	CAN1_Transmit(CAN_CONTROL_TYPE << 3, Can1_Tx_Buffer, 8);
}

void Driver_Data_Handler(CanRxMsg* rx_msg)
{

}
