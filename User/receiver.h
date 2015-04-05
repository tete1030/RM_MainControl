#ifndef __RECEIVER_H__
#define __RECEIVER_H__
#include <stdint.h>

void Receiver_Configuration(void);
extern uint16_t sbus_channel_temp[15];  //  temp sbus decode channel data
extern uint16_t  radio_yuntai_temp[4];  //
//extern uint16_t movespeed_1;
int Aimed_Position_Yaw_PID(int current_pos,int desired_pos);
int Aimed_Position_Pitch_PID(int current_pos,int desired_pos);
#endif
