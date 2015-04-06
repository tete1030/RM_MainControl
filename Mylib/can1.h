#ifndef __CAN1_H__
#define __CAN1_H__

#define CAN_CONTROL_IDENTIFIER 0x66
#define CAN_DRIVER_LEFT_FRONT_ADDR 0x1
#define CAN_DRIVER_RIGHT_FRONT_ADDR 0x2
#define CAN_DRIVER_LEFT_END_ADDR 0x3
#define CAN_DRIVER_RIGHT_END_ADDR 0x4

void CAN1_Configuration(void);
void Conmmunication(int16_t Move_Speed_X,int16_t Move_Speed_Y,int Rotate);

#endif 
