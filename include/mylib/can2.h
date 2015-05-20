//
// Created by Texot Qexoq on 5/16/15.
//

#ifndef __CAN2_H__
#define __CAN2_H__

void CAN2_Configuration(void (*send_handler)(uint16_t, int8_t), void (*receive_handler)(CanRxMsg*), uint16_t IdHigh, uint16_t IdHighMask, uint16_t IdLow, uint16_t IdLowMask);
int8_t CAN2_Transmit(uint16_t id, uint16_t addr, char* data, uint8_t size);
int8_t CAN2_AsyncTransmit(uint16_t id, uint16_t addr, char* data, uint8_t size);

#endif //__CAN2_H__
