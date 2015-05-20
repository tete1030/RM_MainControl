#include <stdio.h>
#include "stm32f4xx_can.h"
#include "stm32f4xx.h"
#include "receiver.h"
#include "can2.h"
#include "can_packet.h"
#include "stm32f4xx_it.h"
#include "gimbal.h"
#include "main.h"

#define CAN_GIMBAL_ADDR 0x0

#pragma pack(push,1)
struct Gimbal_Maincontrol_Packet_Others
{
    int8_t friction_state;
    int8_t friction_ready_state;
    int8_t shooter_state;
    uint16_t pitch_angle;
    uint16_t yaw_angle;
};

struct Gimbal_Maincontrol_Packet_Accel
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Gimbal_Maincontrol_Packet_Gyro
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Gimbal_Maincontrol_Packet_Yaw_Pitch
{
    float yaw;
    float pitch;
};

struct Gimbal_Maincontrol_Packet_Roll
{
    float roll;
};

struct Maincontrol_Gimbal_Packet_Set_Speed
{
    float yaw;
    float pitch;
};

struct Maincontrol_Gimbal_Packet_Set_Friction
{
    uint8_t enable;
};

struct Maincontrol_Gimbal_Packet_Set_Shooter
{
    uint8_t enable;
};

struct Maincontrol_Gimbal_Packet_Set_Laser
{
    uint8_t enable;
};

struct Maincontrol_Gimbal_Packet_Set_Yaw_Pitch
{
    float yaw;
    float pitch;
};

#pragma pack(pop)

void Gimbal_Configuration(void)
{
	extern void Gimbal_Can_Receive_Handler(CanRxMsg* rx_msg);
	extern void Gimbal_Can_Send_Handler(uint16_t id, int8_t code);
	CAN2_Configuration(Gimbal_Can_Send_Handler, Gimbal_Can_Receive_Handler,
                       CAN_PACKET_DESTTYPE_CENTER | CAN_PACKET_PLACETYPE_ALLINONE | CAN_GIMBAL_ADDR,
					   CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_SOURCEADDR_ALL,
                       CAN_PACKET_DESTTYPE_CENTER | CAN_PACKET_PLACETYPE_SPECIFIC | CAN_GIMBAL_ADDR,
					   CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_SOURCEADDR_ALL);
}

void Gimbal_Can_Receive_Handler(CanRxMsg* rx_msg)
{
    int16_t type,id;

    type = (uint16_t) (rx_msg->StdId & CAN_PACKET_TYPE_MASK_DATATYPE);
    id = (uint16_t) (rx_msg->StdId & CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT);

    if(Car_Status_Lock()) {

        // TODO: rewrite other can packet seal methods
        switch (type) {
            case CAN_PACKET_CENTER_DATATYPE_GIMBAL_OTHERS:
                car_status.remote_friction = ((struct Gimbal_Maincontrol_Packet_Others *)(rx_msg->Data))->friction_state;
                car_status.remote_friction_ready = ((struct Gimbal_Maincontrol_Packet_Others *)(rx_msg->Data))->friction_ready_state;
                car_status.remote_shooter = ((struct Gimbal_Maincontrol_Packet_Others *)(rx_msg->Data))->shooter_state;
                car_status.remote_angle_gimbal_motor_pitch = ((struct Gimbal_Maincontrol_Packet_Others *)(rx_msg->Data))->pitch_angle;
                car_status.remote_angle_gimbal_motor_yaw = ((struct Gimbal_Maincontrol_Packet_Others *)(rx_msg->Data))->yaw_angle;
                break;
            case CAN_PACKET_CENTER_DATATYPE_GIMBAL_ACCEL:
                car_status.remote_accel.x = ((struct Gimbal_Maincontrol_Packet_Accel *)(rx_msg->Data))->x;
                car_status.remote_accel.y = ((struct Gimbal_Maincontrol_Packet_Accel *)(rx_msg->Data))->y;
                car_status.remote_accel.z = ((struct Gimbal_Maincontrol_Packet_Accel *)(rx_msg->Data))->z;
                break;
            case CAN_PACKET_CENTER_DATATYPE_GIMBAL_GYRO:
                car_status.remote_gyro.x = ((struct Gimbal_Maincontrol_Packet_Gyro *)(rx_msg->Data))->x;
                car_status.remote_gyro.y = ((struct Gimbal_Maincontrol_Packet_Gyro *)(rx_msg->Data))->y;
                car_status.remote_gyro.z = ((struct Gimbal_Maincontrol_Packet_Gyro *)(rx_msg->Data))->z;
                break;
            case CAN_PACKET_CENTER_DATATYPE_GIMBAL_YAW_PITCH:
                car_status.remote_angle_gimbal_pitch = ((struct Gimbal_Maincontrol_Packet_Yaw_Pitch *)(rx_msg->Data))->pitch;
                car_status.remote_angle_gimbal_yaw = ((struct Gimbal_Maincontrol_Packet_Yaw_Pitch *)(rx_msg->Data))->yaw;
                break;
            case CAN_PACKET_CENTER_DATATYPE_GIMBAL_ROLL:
                car_status.remote_angle_gimbal_roll = ((struct Gimbal_Maincontrol_Packet_Roll *)(rx_msg->Data))->roll;
                break;
            default:
                ;// do nothing
        }

        Car_Status_Unlock();
    }

}

void Gimbal_Can_Send_Handler(uint16_t id, int8_t code)
{
    if(code == 0)printf("%u send failed from can2\r\n", id);
}

#define GIMBAL_PACKET_SET_SPEED 0x0

void Gimbal_Set_Speed(float speed_yaw, float speed_pitch)
{
    static struct Maincontrol_Gimbal_Packet_Set_Speed gpss;
    gpss.yaw = speed_yaw;
    gpss.pitch = speed_pitch;
    CAN2_AsyncTransmit(GIMBAL_PACKET_SET_SPEED,
                       CAN_PACKET_DESTTYPE_GIMBAL |
                       CAN_PACKET_GIMBAL_DATATYPE_SET_SPEED |
                       CAN_PACKET_PLACETYPE_SPECIFIC |
                       CAN_ADDR,
                       (char*) &gpss,
                       sizeof(gpss)
    );
}

#define GIMBAL_PACKET_SET_FRICTION 0x1

void Gimbal_Set_Friction(uint8_t enable)
{
    static struct Maincontrol_Gimbal_Packet_Set_Friction gpsf;
    gpsf.enable = enable;
    CAN2_AsyncTransmit(GIMBAL_PACKET_SET_FRICTION,
                       CAN_PACKET_DESTTYPE_GIMBAL |
                       CAN_PACKET_GIMBAL_DATATYPE_SET_FRICTION |
                       CAN_PACKET_PLACETYPE_SPECIFIC |
                       CAN_ADDR,
                       (char*)&gpsf,
                       sizeof(gpsf)
    );
}

#define GIMBAL_PACKET_SET_SHOOTER 0x2

void Gimbal_Set_Shooter(uint8_t enable)
{
    static struct Maincontrol_Gimbal_Packet_Set_Shooter gpss;
    gpss.enable = enable;
    CAN2_AsyncTransmit(GIMBAL_PACKET_SET_SHOOTER,
                       CAN_PACKET_DESTTYPE_GIMBAL |
                       CAN_PACKET_GIMBAL_DATATYPE_SET_SHOOTER |
                       CAN_PACKET_PLACETYPE_SPECIFIC |
                       CAN_ADDR,
                       (char*) &gpss,
                       sizeof(gpss)
    );
}

#define GIMBAL_PACKET_SET_LASER 0x3

void Gimbal_Set_Laser(uint8_t enable)
{
    static struct Maincontrol_Gimbal_Packet_Set_Laser gpsl;
    gpsl.enable = enable;
    CAN2_AsyncTransmit(GIMBAL_PACKET_SET_LASER,
                       CAN_PACKET_DESTTYPE_GIMBAL |
                       CAN_PACKET_GIMBAL_DATATYPE_SET_LASER |
                       CAN_PACKET_PLACETYPE_SPECIFIC |
                       CAN_ADDR,
                       (char*) &gpsl,
                       sizeof(gpsl)
    );
}

#define GIMBAL_PACKET_SET_YAW_PITCH 0x4

void Gimbal_Set_Yaw_Pitch(float yaw, float pitch)
{
    static struct Maincontrol_Gimbal_Packet_Set_Yaw_Pitch mgpsyp;
    mgpsyp.yaw = yaw;
    mgpsyp.pitch = pitch;
    CAN2_AsyncTransmit(GIMBAL_PACKET_SET_YAW_PITCH,
                       CAN_PACKET_DESTTYPE_GIMBAL |
                       CAN_PACKET_GIMBAL_DATATYPE_SET_YAW_PITCH |
                       CAN_PACKET_PLACETYPE_SPECIFIC |
                       CAN_ADDR,
                       (char*) &mgpsyp,
                       sizeof(mgpsyp)
    );
}