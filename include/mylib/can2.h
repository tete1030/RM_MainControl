#ifndef __CAN2_H__
#define __CAN2_H__

void CAN2_Configuration(void);
extern int32_t angle;
extern int16_t Gim_yaw, Gim_pitch;
extern uint8_t Mode;
extern uint8_t Shoot;

void SendGimbalPosition(uint16_t Gim_yaw, uint16_t Gim_pitch, uint8_t Shoot,
		uint8_t Mode);

void SendGimbalPosition_mouse(int16_t Gim_yaw, int16_t Gim_pitch, uint8_t Shoot,
		uint8_t Mode);
#endif 
