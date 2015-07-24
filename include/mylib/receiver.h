#ifndef __RECEIVER_H__
#define __RECEIVER_H__
#include <stdint.h>

#define RECEIVER_CHANNEL_VALUE_START 364
#define RECEIVER_CHANNEL_VALUE_END 1684
#define RECEIVER_CHANNEL_VALUE_CENTER 1024
#define RECEIVER_CHANNEL_VALUE_RANGE_SIZE (RECEIVER_CHANNEL_VALUE_END - RECEIVER_CHANNEL_VALUE_CENTER)


#define RECEIVER_PACKET_KEY_PRESSED_W (1 << 0)
#define RECEIVER_PACKET_KEY_PRESSED_S (1 << 1)
#define RECEIVER_PACKET_KEY_PRESSED_A (1 << 2)
#define RECEIVER_PACKET_KEY_PRESSED_D (1 << 3)
#define RECEIVER_PACKET_KEY_PRESSED_Q (1 << 4)
#define RECEIVER_PACKET_KEY_PRESSED_E (1 << 5)
#define RECEIVER_PACKET_KEY_PRESSED_SHIFT (1 << 6)
#define RECEIVER_PACKET_KEY_PRESSED_CONTROL (1 << 7)

// possible key according to RoboMasters通信协议v1.0.6.pdf
#define RECEIVER_PACKET_KEY_PRESSED_R (1 << 8)
#define RECEIVER_PACKET_KEY_PRESSED_F (1 << 9)
#define RECEIVER_PACKET_KEY_PRESSED_G (1 << 10)
#define RECEIVER_PACKET_KEY_PRESSED_Z (1 << 11)
#define RECEIVER_PACKET_KEY_PRESSED_X (1 << 12)
#define RECEIVER_PACKET_KEY_PRESSED_C (1 << 13)
#define RECEIVER_PACKET_KEY_PRESSED_V (1 << 14)
#define RECEIVER_PACKET_KEY_PRESSED_B (1 << 15)

typedef struct Receiver_Packet
{
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint8_t s1;
    uint8_t s2;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;

    uint16_t key_pressd;

    uint16_t reserved;
} Receiver_Packet;

void Receiver_Configuration(void);
int8_t Receiver_Get_New_Packet(Receiver_Packet *rp);

#endif
