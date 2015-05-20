//
// Created by Texot Qexoq on 5/16/15.
//
#include "main.h"
#include "driver.h"
#include "execute.h"
#include "receiver.h"
#include "gimbal.h"


#define MOUSE_YAW_SPEED_RATIO 100

Receiver_Packet packet_to_execute;

void Execute_Receiver_Command() {
    int16_t vx, vy;
    float w0;

    if (!Receiver_Get_New_Packet(&packet_to_execute))
    {
        // TODO: after a while keep the monster safe and alert
    }

    // up
    if(packet_to_execute.s2 == 1)
    {
        // TODO: controlled by remoter
        vx = (((float)packet_to_execute.ch1 - RECEIVER_CHANNEL_VALUE_CENTER) * CAR_MAX_X_SPEED) / RECEIVER_CHANNEL_VALUE_RANGE_SIZE;
        vy = (((float)packet_to_execute.ch0 - RECEIVER_CHANNEL_VALUE_CENTER) * CAR_MAX_Y_SPEED) / RECEIVER_CHANNEL_VALUE_RANGE_SIZE;
        w0 = (((float)packet_to_execute.ch2 - RECEIVER_CHANNEL_VALUE_CENTER) * CAR_MAX_W0_SPEED ) / RECEIVER_CHANNEL_VALUE_RANGE_SIZE;

        Driver_Set_Speed(vx, vy, -w0);

    }
    // middle
    else if(packet_to_execute.s2 == 3)
    {
        // controlled by mouse and keyboard
        Car_Status_Lock();
        // turn on, must when friction being ready
        if(packet_to_execute.mouse_left && !car_status.local_shooter && car_status.remote_friction_ready) {
            // TODO: unlock friction
            car_status.local_friction_locked = 1;
            car_status.local_shooter = 1;
            Gimbal_Set_Shooter(1);
        }
        else if(!packet_to_execute.mouse_left && car_status.local_shooter) {
            car_status.local_shooter = 0;
            Gimbal_Set_Shooter(0);
        }

        if (packet_to_execute.mouse_right && !car_status.local_friction) {
            car_status.local_friction = 1;
            Gimbal_Set_Friction(1);
        }
        else if (!packet_to_execute.mouse_right && car_status.local_friction && !car_status.local_friction_locked) {
            Gimbal_Set_Friction(0);
        }


        Gimbal_Set_Speed((float) packet_to_execute.mouse_x / MOUSE_YAW_SPEED_RATIO , (float) packet_to_execute.mouse_y / MOUSE_YAW_SPEED_RATIO);
        // TODO: remaining

        Car_Status_Unlock();

    }
    // down
    else if(packet_to_execute.s2 == 2)
    {
        // TODO: release control

    }

}
