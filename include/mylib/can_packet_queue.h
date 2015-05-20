//
// Created by Texot Qexoq on 5/19/15.
//

#ifndef __CAN_PACKET_QUEUE_H__
#define __CAN_PACKET_QUEUE_H__

#include <stdint.h>

typedef struct CAN_Packet_Queue
{
    uint16_t id;
    uint16_t addr;
    int8_t data[8];
    uint8_t size;
} CAN_Packet_Queue;

#endif //__CAN_PACKET_QUEUE_H__
