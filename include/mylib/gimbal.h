#ifndef __GIMBAL_H__
#define __GIMBAL_H__



void Gimbal_Configuration(void);
void Gimbal_Set_Speed(float speed_yaw, float speed_pitch);
void Gimbal_Set_Friction(uint8_t enable);
void Gimbal_Set_Shooter(uint8_t enable);
void Gimbal_Set_Laser(uint8_t enable);

/*
extern int32_t angle;
extern int16_t Gim_yaw, Gim_pitch;
extern uint8_t Mode;
extern uint8_t Shoot;

void SendGimbalPosition(uint16_t Gim_yaw, uint16_t Gim_pitch, uint8_t Shoot,
		uint8_t Mode);

void SendGimbalPosition_mouse(int16_t Gim_yaw, int16_t Gim_pitch, uint8_t Shoot,
		uint8_t Mode);
		*/
#endif 
