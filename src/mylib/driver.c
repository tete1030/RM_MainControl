#include <stdio.h>
#include <mylib/driver.h>
#include "stm32f4xx.h"
#include "driver.h"
#include "can1.h"
#include "main.h"
#include "can_packet.h"
#include "debug.h"

#define DRIVER_ENCODER_SAMPLE_DURATION 5
#define DRIVER_ENCODER_SPEED_FACTOR (1000/DRIVER_ENCODER_SAMPLE_DURATION)

#define CAR_WIDTH_PLUS_LENGTH_N_DIV_2 ((CAR_WIDTH + CAR_LENGTH) / 2)

// TODO: need to be adapted to arm_cc
#pragma pack(push, 1)

struct Driver_Packet_Status
{
    int16_t encoder_value;
};

struct Driver_Packet_Stdout
{
    char content[8];
};

struct Driver_Packet_Set_Enable
{
    uint8_t sel_enable;
};

struct Driver_Packet_Set_Speed
{
    int16_t speed_left_front;
    int16_t speed_right_front;
    int16_t speed_left_end;
    int16_t speed_right_end;
};

struct Driver_Packet_Set_Wheel_Speed
{
    int16_t speed;
};

struct Driver_Packet_Set_Configuration
{
    int8_t motor_config_type;
    union
    {
        uint16_t max_speed;
        struct
        {
            uint8_t vp;
            uint8_t vi;
            uint8_t vd;
        } pid;
        uint16_t speed_step;
    } data;
};
#define sizeof_DPSC_max_speed 3
#define sizeof_DPSC_pid 4
#define sizeof_DPSC_speed_step 3


#pragma pack(pop)




void Driver_Configuration()
{
	extern void Driver_Can_Receive_Handler(CanRxMsg* rx_msg);
	extern void Driver_Can_Send_Handler(uint16_t id, int8_t code);
	CAN1_Configuration(Driver_Can_Send_Handler, Driver_Can_Receive_Handler,
                       CAN_PACKET_DESTTYPE_CENTER | CAN_PACKET_PLACETYPE_ALLINONE | CAN_DRIVER_SELECT_ALL_ADDR,
                       CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_SOURCEADDR_ALL,
                       CAN_PACKET_DESTTYPE_CENTER | CAN_PACKET_PLACETYPE_SPECIFIC | CAN_DRIVER_SELECT_ALL_ADDR,
                       CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_SOURCEADDR_ALL);

}

#define DRIVER_PACKET_ID_ENABLE 0x1

void Driver_Set_Enable(uint8_t motor_sel, uint8_t motor_enable_sel)
{
	static struct Driver_Packet_Set_Enable dpse;
    dpse.sel_enable = motor_enable_sel;
	CAN1_AsyncTransmit(DRIVER_PACKET_ID_ENABLE,
                       (uint16_t)
                               (CAN_PACKET_DESTTYPE_DRIVER |
                                CAN_PACKET_DRIVER_DATATYPE_ENABLE |
                                CAN_PACKET_PLACETYPE_ALLINONE |
                                       motor_sel),
                       (char*) &dpse,
                       sizeof(dpse)
    );
}

#define DRIVER_PACKET_ID_SPEED 0x2

// Set Car Speed
// Unit: vx, vy is mm/s, w0 is rad/s
void Driver_Set_Speed(int16_t vx, int16_t vy, float w0)
{
	static struct Driver_Packet_Set_Speed dpss;

	float p = (float) (vx + vy) * CAR_ENCODER_NUM;
	float q = (float) (vx - vy) * CAR_ENCODER_NUM;
	float m = w0 * CAR_WIDTH_PLUS_LENGTH_N_DIV_2 * CAR_ENCODER_NUM;
	float n = (float) (2 * PI * CAR_WHEEL_RADIUS * DRIVER_ENCODER_SPEED_FACTOR);

	dpss.speed_left_front = (int16_t)((p - m) / n);
	dpss.speed_right_front = (int16_t)((q + m) / n);
	dpss.speed_left_end = (int16_t)((q - m) / n);
	dpss.speed_right_end = (int16_t)((p + m) / n);

	//printf("%d, %d, %d, %d\r\n", wlf, wrf, wle, wre);

	CAN1_AsyncTransmit(DRIVER_PACKET_ID_SPEED,
                       CAN_PACKET_DESTTYPE_DRIVER |
                       CAN_PACKET_DRIVER_DATATYPE_CONTROL |
                       CAN_PACKET_PLACETYPE_ALLINONE |
                       CAN_DRIVER_SELECT_ALL_ADDR,
                       (char*) &dpss,
                       sizeof(dpss));
}

#define DRIVER_PACKET_ID_WHEEL_SPEED 0x3

void Driver_Set_Wheel_Speed(uint8_t motor_sel, int16_t destSpeed)
{
    static struct Driver_Packet_Set_Wheel_Speed dpsws;
    dpsws.speed = destSpeed;
	CAN1_AsyncTransmit(DRIVER_PACKET_ID_WHEEL_SPEED,
                       (uint16_t)
                               (CAN_PACKET_DESTTYPE_DRIVER |
                                CAN_PACKET_DRIVER_DATATYPE_CONTROL |
                                CAN_PACKET_PLACETYPE_SPECIFIC |
                                       motor_sel),
                       (char*) &dpsws,
                       sizeof(dpsws));
}

#define DRIVER_PACKET_ID_CONFIG 0x4

void Driver_Set_Configuration(uint8_t motor_sel, Motor_Config_Type mct, void* value)
{
    static struct Driver_Packet_Set_Configuration dpsc;
	uint8_t send_size = 0;

    dpsc.motor_config_type = (int8_t)mct;
	switch(mct)
	{
	case MCT_Max_Speed:
        dpsc.data.max_speed = *((uint16_t*)value);
        send_size = sizeof_DPSC_max_speed;
		break;
	case MCT_PID:
        dpsc.data.pid.vp = ((uint8_t*)value)[0];
        dpsc.data.pid.vi = ((uint8_t*)value)[1];
        dpsc.data.pid.vd = ((uint8_t*)value)[2];
		send_size = sizeof_DPSC_pid;
		break;
	case MCT_Speed_Step:
		dpsc.data.speed_step = *((uint16_t*)value);
		send_size = sizeof_DPSC_speed_step;
		break;
	}

	if(send_size > 0)
	{
		CAN1_AsyncTransmit(DRIVER_PACKET_ID_CONFIG,
                           (uint8_t)
                                   (CAN_PACKET_DESTTYPE_DRIVER |
                                    CAN_PACKET_DRIVER_DATATYPE_CONFIG |
                                    CAN_PACKET_PLACETYPE_SPECIFIC |
                                           motor_sel),
                           (char*) &dpsc,
                           send_size);
	}

}



void Driver_Can_Receive_Handler(CanRxMsg* rx_msg)
{
	int16_t type,id;

    type = (uint16_t) (rx_msg->StdId & CAN_PACKET_TYPE_MASK_DATATYPE);
    id = (uint16_t) (rx_msg->StdId & CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT);


        if (type == CAN_PACKET_CENTER_DATATYPE_DRIVER_STDOUT) {

        }
        else if (type == CAN_PACKET_CENTER_DATATYPE_DRIVER_STATUS) {
            if(Car_Status_Lock()) {
                switch(id)
                {
                    case CAN_DRIVER_LEFT_FRONT_ADDR:
                        car_status.remote_speed_motor_left_front =
                                ((struct Driver_Packet_Status *)rx_msg->Data)->encoder_value;
                    case CAN_DRIVER_RIGHT_FRONT_ADDR:
                        car_status.remote_speed_motor_right_front =
                                ((struct Driver_Packet_Status *)rx_msg->Data)->encoder_value;
                    case CAN_DRIVER_LEFT_END_ADDR:
                        car_status.remote_speed_motor_left_end =
                                ((struct Driver_Packet_Status *)rx_msg->Data)->encoder_value;
                    case CAN_DRIVER_RIGHT_END_ADDR:
                        car_status.remote_speed_motor_right_end =
                                ((struct Driver_Packet_Status *)rx_msg->Data)->encoder_value;
                    default:
                        ;
                }
                Car_Status_Unlock();
            }
        }


}


void Driver_Can_Send_Handler(uint16_t id, int8_t code)
{
	if(code == 0)printf("%u send failed from can1\r\n", id);
}
