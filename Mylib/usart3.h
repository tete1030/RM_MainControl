#ifndef __USART1_H__
#define __USART1_H__

void USART3_Configuration(void);
void USART3_SendChar(unsigned char b);
extern unsigned char Auto_Aim_Cmd;
extern int Gim_yaw_tmp,Gim_pitch_tmp;
#endif
