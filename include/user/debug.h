#ifndef __DEBUG_H__
#define __DEBUG_H__

#define MAINCONTROL_TYPE_PRINTF 0x1
#define MAINCONTROL_TYPE_STATUS 0x2

#define MAINCONTROL_DATA_SOURCE_MAINCONTROL 0x1
#define MAINCONTROL_DATA_SOURCE_DRIVER_LF 0x2
#define MAINCONTROL_DATA_SOURCE_DRIVER_RF 0x3
#define MAINCONTROL_DATA_SOURCE_DRIVER_LE 0x4
#define MAINCONTROL_DATA_SOURCE_DRIVER_RE 0x5
#define MAINCONTROL_DATA_SOURCE_GIMBAL 0x6

void Debug_Configuration(void);
int Debug_Transmit(int8_t source, int8_t type, char* data, uint8_t size);
void Debug_Execute_Computer_Command(void);

#endif
