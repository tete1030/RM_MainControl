//
// Created by Texot Qexoq on 5/16/15.
//
#include "main.h"
#include "driver.h"
#include "execute.h"
#include "receiver.h"
#include "gimbal.h"
#include "ticker.h"

#define REMOTER_SPEED_STEP 100
#define COMPUTER_SPEED_CONTROL_STEP 100
#define COMPUTER_SPEED_RELEASE_STEP 500

typedef enum Control_Mode
{
    Control_Mode_Remoter,
    Control_Mode_Computer,
    Control_Mode_Release
} Control_Mode;


Control_Mode last_control_mode = Control_Mode_Release;

int16_t speed_x = 0;
int16_t speed_y = 0;
uint8_t friction_state = 0;
uint8_t last_friction_switch_state = 0;

volatile int16_t car_mech_angle_yaw = 0;

#define MOUSE_YAW_SPEED_RATIO 15
#define MOUSE_PITCH_SPEED_RATIO 40

Receiver_Packet packet_to_execute;

#define UINT64_MAX 0xffffffffffffffff

uint64_t first_no_packet_tick = UINT64_MAX;
uint32_t ms_tickcount;

void Execute_Init(void)
{
	ms_tickcount = Ticker_Get_MS_Tickcount();
}

void Execute_Do_Receiver_Command() {

    uint8_t cur_friction_switch_state = 0;
    float yaw, pitch;
    float w0;
    int16_t new_speed_x, new_speed_y;
    uint64_t cur_tick;

    if (!Receiver_Get_New_Packet(&packet_to_execute))
    {
    	cur_tick = Ticker_Get_Tick();
    	if(first_no_packet_tick != UINT64_MAX)
    	{
    		if((cur_tick - first_no_packet_tick) > 100 * ms_tickcount)
    		{
    			Gimbal_Set_Friction(0);
				Gimbal_Set_Shooter(0);
				Gimbal_Set_Speed(0, 0);
				Driver_Set_Speed(0, 0, 0);

				speed_x = 0;
				speed_y = 0;
				last_friction_switch_state = 0;

    		}
    	}
    	else
    		first_no_packet_tick = cur_tick;
    	return;

    }

    first_no_packet_tick = UINT64_MAX;

    if(Maincontrol_Get_Enable_Control_State() == 0)
	{
    	Gimbal_Set_Friction(0);
		Gimbal_Set_Shooter(0);
		Gimbal_Set_Speed(0, 0);
		Driver_Set_Speed(0, 0, 0);

		speed_x = 0;
		speed_y = 0;
		last_friction_switch_state = 0;
    	return;
	}

    // up
    if(packet_to_execute.s2 == 1)
    {
        if(last_control_mode != Control_Mode_Remoter)
        {
            last_friction_switch_state = 0;
            last_control_mode = Control_Mode_Remoter;
        }

        // TODO: controlled by remoter
        new_speed_x = (((float)packet_to_execute.ch1 - RECEIVER_CHANNEL_VALUE_CENTER) * CAR_MAX_X_HIGH_SPEED) / RECEIVER_CHANNEL_VALUE_RANGE_SIZE;
        new_speed_y = (((float)packet_to_execute.ch0 - RECEIVER_CHANNEL_VALUE_CENTER) * CAR_MAX_Y_HIGH_SPEED) / RECEIVER_CHANNEL_VALUE_RANGE_SIZE;
        w0 = (((float)packet_to_execute.ch2 - RECEIVER_CHANNEL_VALUE_CENTER) * CAR_MAX_W0_SPEED ) / RECEIVER_CHANNEL_VALUE_RANGE_SIZE;

        yaw = (((float)packet_to_execute.ch2 - RECEIVER_CHANNEL_VALUE_CENTER) * CAR_MAX_YAW_SPEED ) / RECEIVER_CHANNEL_VALUE_RANGE_SIZE;
        pitch = (((float)packet_to_execute.ch3 - RECEIVER_CHANNEL_VALUE_CENTER) * CAR_MAX_PITCH_SPEED ) / RECEIVER_CHANNEL_VALUE_RANGE_SIZE;

        if(new_speed_x > speed_x + REMOTER_SPEED_STEP) speed_x += REMOTER_SPEED_STEP;
        else if(new_speed_x < speed_x - REMOTER_SPEED_STEP) speed_x -= REMOTER_SPEED_STEP;
        else speed_x = new_speed_x;

        if(new_speed_y > speed_y + REMOTER_SPEED_STEP) speed_y += REMOTER_SPEED_STEP;
        else if(new_speed_y < speed_y - REMOTER_SPEED_STEP) speed_y -= REMOTER_SPEED_STEP;
        else speed_y = new_speed_y;

        if(speed_x > CAR_MAX_X_SPEED) speed_x = CAR_MAX_X_SPEED;
        if(speed_x < -CAR_MAX_X_SPEED) speed_x = -CAR_MAX_X_SPEED;

        if(speed_y > CAR_MAX_Y_SPEED) speed_y = CAR_MAX_Y_SPEED;
        if(speed_y < -CAR_MAX_Y_SPEED) speed_y = -CAR_MAX_Y_SPEED;

        cur_friction_switch_state = packet_to_execute.s1 == 1? 1 : 0;
        if(!last_friction_switch_state && cur_friction_switch_state)
            friction_state = (~friction_state) & 1;

#if (defined CAR_2)
        Gimbal_Set_Speed(0, pitch);
        //Driver_Set_Speed(speed_x, speed_y, Driver_Angle_Control(car_mech_angle_yaw));
        Driver_Set_Speed(speed_x, speed_y, w0);
#elif (defined CAR_3)
        Gimbal_Set_Speed(0, -pitch);
        //Driver_Set_Speed(speed_x, speed_y, Driver_Angle_Control(car_mech_angle_yaw));
        Driver_Set_Speed(speed_x, speed_y, w0);
#elif (defined CAR_1)
        Driver_Set_Speed(speed_x, speed_y, w0);
#endif
        Gimbal_Set_Friction(friction_state);
        Gimbal_Set_Shooter((friction_state && packet_to_execute.s1 == 2) ? 1 : 0);

        last_friction_switch_state = cur_friction_switch_state;
    }
    // middle
    else if(packet_to_execute.s2 == 3)
    {
        if(last_control_mode != Control_Mode_Computer)
        {
            last_friction_switch_state = 0;
            last_control_mode = Control_Mode_Computer;
        }

        if(!(packet_to_execute.key_pressd & (RECEIVER_PACKET_KEY_PRESSED_W | RECEIVER_PACKET_KEY_PRESSED_S)))
        {
            if(speed_x > 0)
            {
                speed_x -= COMPUTER_SPEED_RELEASE_STEP;
                if(speed_x < 0) speed_x = 0;
            }
            else if(speed_x < 0)
            {
                speed_x += COMPUTER_SPEED_RELEASE_STEP;
                if(speed_x > 0) speed_x = 0;
            }
        }
        else
        {
            if(packet_to_execute.key_pressd & RECEIVER_PACKET_KEY_PRESSED_W)
                speed_x += COMPUTER_SPEED_CONTROL_STEP;
            if(packet_to_execute.key_pressd & RECEIVER_PACKET_KEY_PRESSED_S)
                speed_x -= COMPUTER_SPEED_CONTROL_STEP;
        }

        if(!(packet_to_execute.key_pressd & (RECEIVER_PACKET_KEY_PRESSED_A | RECEIVER_PACKET_KEY_PRESSED_D)))
        {
            if(speed_y > 0)
            {
                speed_y -= COMPUTER_SPEED_RELEASE_STEP;
                if(speed_y < 0) speed_y = 0;
            }
            else if(speed_y < 0)
            {
                speed_y += COMPUTER_SPEED_RELEASE_STEP;
                if(speed_y > 0) speed_y = 0;
            }
        }
        else
        {
            if(packet_to_execute.key_pressd & RECEIVER_PACKET_KEY_PRESSED_A)
                speed_y -= COMPUTER_SPEED_CONTROL_STEP;
            if(packet_to_execute.key_pressd & RECEIVER_PACKET_KEY_PRESSED_D)
                speed_y += COMPUTER_SPEED_CONTROL_STEP;
        }

        if(packet_to_execute.key_pressd & RECEIVER_PACKET_KEY_PRESSED_SHIFT)
        {
			if(speed_x > CAR_MAX_X_HIGH_SPEED) speed_x = CAR_MAX_X_HIGH_SPEED;
			if(speed_x < -CAR_MAX_X_HIGH_SPEED) speed_x = -CAR_MAX_X_HIGH_SPEED;

			if(speed_y > CAR_MAX_Y_HIGH_SPEED) speed_y = CAR_MAX_Y_HIGH_SPEED;
			if(speed_y < -CAR_MAX_Y_HIGH_SPEED) speed_y = -CAR_MAX_Y_HIGH_SPEED;
        }
        else
        {
        	if(speed_x > CAR_MAX_X_SPEED + COMPUTER_SPEED_CONTROL_STEP) speed_x -= COMPUTER_SPEED_CONTROL_STEP;
        	else if(speed_x > CAR_MAX_X_SPEED) speed_x = CAR_MAX_X_SPEED;

			if(speed_x < -CAR_MAX_X_SPEED - COMPUTER_SPEED_CONTROL_STEP) speed_x += COMPUTER_SPEED_CONTROL_STEP;
			else if(speed_x < -CAR_MAX_X_SPEED) speed_x = -CAR_MAX_X_SPEED;

			if(speed_y > CAR_MAX_Y_SPEED + COMPUTER_SPEED_CONTROL_STEP) speed_y -= COMPUTER_SPEED_CONTROL_STEP;
			else if(speed_y > CAR_MAX_Y_SPEED) speed_y = CAR_MAX_Y_SPEED;
			if(speed_y < -CAR_MAX_Y_SPEED - COMPUTER_SPEED_CONTROL_STEP) speed_y += COMPUTER_SPEED_CONTROL_STEP;
			else if(speed_y < -CAR_MAX_Y_SPEED) speed_y = -CAR_MAX_Y_SPEED;
        }

        cur_friction_switch_state = packet_to_execute.mouse_right;
        if(!last_friction_switch_state && cur_friction_switch_state)
            friction_state = (~friction_state) & 1;

        // controlled by mouse and keyboard


#if (defined CAR_2)
        //Gimbal_Set_Speed((float) packet_to_execute.mouse_x / MOUSE_YAW_SPEED_RATIO, (float) -packet_to_execute.mouse_y / MOUSE_PITCH_SPEED_RATIO);
        Gimbal_Set_Speed(0, (float) -packet_to_execute.mouse_y / MOUSE_PITCH_SPEED_RATIO);
        Driver_Set_Speed(speed_x, speed_y, (float) packet_to_execute.mouse_x / (MOUSE_YAW_SPEED_RATIO ));//* 360 / PI));//Driver_Angle_Control(car_mech_angle_yaw));
#elif (defined CAR_3)
        Gimbal_Set_Speed(0, (float) packet_to_execute.mouse_y / MOUSE_PITCH_SPEED_RATIO);
        //Driver_Set_Speed(speed_x, speed_y, Driver_Angle_Control(car_mech_angle_yaw));
        Driver_Set_Speed(speed_x, speed_y, (float) packet_to_execute.mouse_x / (MOUSE_YAW_SPEED_RATIO ));//* 360 / PI));//Driver_Angle_Control(car_mech_angle_yaw));
#elif (defined CAR_1)
        Driver_Set_Speed(speed_x, speed_y, packet_to_execute.mouse_x / MOUSE_YAW_SPEED_RATIO);
#endif
        Gimbal_Set_Friction(friction_state);
        Gimbal_Set_Shooter((friction_state && packet_to_execute.mouse_left) ? 1 : 0);

        last_friction_switch_state = cur_friction_switch_state;
        // TODO: remaining
    }
    // down
    else// if(packet_to_execute.s2 == 2)
    {
    	if(last_control_mode != Control_Mode_Release)
		{
			last_friction_switch_state = 0;
			last_control_mode = Control_Mode_Release;
		}
        Gimbal_Config_Friction(packet_to_execute.ch3);
        // TODO: release control
    }

}
