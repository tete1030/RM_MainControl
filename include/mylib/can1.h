#ifndef __CAN1_H__
#define __CAN1_H__

#include <stdint.h>

void CAN1_Configuration(void (*receive_handler)(CanRxMsg*));
int8_t CAN1_Transmit(int16_t addr, char* data, uint8_t size);

#endif 
