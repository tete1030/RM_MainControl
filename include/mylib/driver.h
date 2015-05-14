#ifndef __DRIVER_H__
#define __DRIVER_H__

typedef enum {
	MCT_Max_Speed = 0,
	MCT_PID = 1,
	MCT_Speed_Step = 2
} Motor_Config_Type;

#define DRIVER_LEFT_FRONT 0x1
#define DRIVER_RIGHT_FRONT 0x2
#define DRIVER_LEFT_END 0x4
#define DRIVER_RIGHT_END 0x8
#define DRIVER_ALL 0xf
#define DRIVER_NONE 0x0

#define CAN_DRIVER_LEFT_FRONT_ADDR DRIVER_LEFT_FRONT
#define CAN_DRIVER_RIGHT_FRONT_ADDR DRIVER_RIGHT_FRONT
#define CAN_DRIVER_LEFT_END_ADDR DRIVER_LEFT_END
#define CAN_DRIVER_RIGHT_END_ADDR DRIVER_RIGHT_END
#define CAN_DRIVER_SELECT_ALL_ADDR DRIVER_ALL

void Driver_Configuration();
void Driver_Set_Enable(uint8_t motor_sel, uint8_t motor_enable_sel);
void Driver_Set_Speed(int16_t vx, int16_t vy, float w0);
void Driver_Set_Wheel_Speed(uint8_t motor_sel, int16_t destSpeed);
void Driver_Set_Configuration(uint8_t motor_sel, Motor_Config_Type mct, void* value);
#endif
