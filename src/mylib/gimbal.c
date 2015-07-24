#include <stdio.h>
#include <stm32f4-stdperiph/stm32f4xx_can.h>
#include "stm32f4xx_can.h"
#include "stm32f4xx.h"
#include "receiver.h"
#include "can2.h"
#include "can_packet.h"
#include "stm32f4xx_it.h"
#include "gimbal.h"
#include "main.h"
#include "execute.h"


#define CAN_GIMBAL_ADDR 0x0

#pragma pack(push,1)
struct Gimbal_Maincontrol_Packet_GM_Mech_Angle_Yaw
{
    int16_t yaw;
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

struct Maincontrol_Gimbal_Packet_Config_Friction
{
    uint16_t remoter_value;
};

struct Maincontrol_Gimbal_Packet_Set_Enable_Control
{
	uint8_t enable;
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

    if(type == CAN_PACKET_CENTER_DATATYPE_STATUS_GM_MECH_ANGLE_YAW)
    {
        car_mech_angle_yaw = ((struct Gimbal_Maincontrol_Packet_GM_Mech_Angle_Yaw *) rx_msg->Data)->yaw;
    }
    else if(type == CAN_PACKET_CENTER_DATATYPE_STATUS_GM_SET_ENABLE_CONTROL)
    {
    	Maincontrol_Set_Enable_Control(((struct Maincontrol_Gimbal_Packet_Set_Enable_Control *) rx_msg->Data)->enable);
    }

}

void Gimbal_Can_Send_Handler(uint16_t id, int8_t code)
{
    if(code != 0)printf("id %u send failed with code %d from can2\r\n", id, code);
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

#define GIMBAL_PACKET_CONFIG_FRICTION 0x5

void Gimbal_Config_Friction(int16_t remoter_value)
{
    static struct Maincontrol_Gimbal_Packet_Config_Friction mgpcf;
    mgpcf.remoter_value = remoter_value;
    CAN2_AsyncTransmit(GIMBAL_PACKET_SET_YAW_PITCH,
                       CAN_PACKET_DESTTYPE_GIMBAL |
                       CAN_PACKET_GIMBAL_DATATYPE_CONFIG_FRICTION |
                       CAN_PACKET_PLACETYPE_SPECIFIC |
                       CAN_ADDR,
                       (char*) &mgpcf,
                       sizeof(mgpcf)
    );
}
