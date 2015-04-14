#ifndef __DRIVER_H__
#define __DRIVER_H__

#define DRIVER_LEFT_FRONT 1
#define DRIVER_RIGHT_FRONT 2
#define DRIVER_LEFT_END 4
#define DRIVER_RIGHT_END 8

void Driver_Configuration();
void Driver_Enable(uint8_t motor_en);
void Driver_Set_Speed(int16_t vx, int16_t vy, float w0);
#endif
