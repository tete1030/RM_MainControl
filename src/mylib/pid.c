//
// Created by Texot Qexoq on 5/18/15.
//

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pid.h"

void PID_Controller_Init(PID_Controller *pidc, PID_Controller_Configuration *pcc)
{
    memcpy(&(pidc->config), pcc, sizeof(PID_Controller_Configuration));

    pidc->sum_kp_ki_kd = pidc->config.kp + pidc->config.ki + pidc->config.kd;
    pidc->sum_kp_2kd = pidc->config.kp + 2 * pidc->config.kd;

    pidc->pre_error[0] = 0;
    pidc->pre_error[1] = 0;
    pidc->pre_error[2] = 0;

    pidc->integral = 0;

    pidc->cur_error_offset = 0;
    pidc->last_output = 0;

}

float _incremental_control(PID_Controller *pidc, float cur_value, float target_value, float other, float *incre_output)
{
    float output;
    pidc->pre_error[pidc->cur_error_offset] = target_value - cur_value;

    /*
    output = pidc->config.kp * (pidc->pre_error[pidc->cur_error_offset] - pidc->pre_error[(pidc->cur_error_offset + 2) % 3])
             + pidc->config.ki * pidc->pre_error[pidc->cur_error_offset]
             + pidc->config.kd *
                        (pidc->pre_error[pidc->cur_error_offset]
                         - 2 * pidc->pre_error[(pidc->cur_error_offset + 2) % 3]
                         + pidc->pre_error[(pidc->cur_error_offset + 1) % 3])
             + pidc->config.ko * other;
             */
    output = pidc->sum_kp_ki_kd * pidc->pre_error[pidc->cur_error_offset]
             - pidc->sum_kp_2kd * pidc->pre_error[(pidc->cur_error_offset + 2) % 3]
             + pidc->config.kd * pidc->pre_error[(pidc->cur_error_offset + 1) % 3]
             + pidc->config.ko * other;

    pidc->cur_error_offset = (uint8_t) ((pidc->cur_error_offset + 1) % 3);

    if(incre_output)
        *incre_output = output;

    output += pidc->last_output;

    if(output > pidc->config.max_output)
        output = pidc->config.max_output;
    else if(output < pidc->config.min_output)
        output = pidc->config.min_output;

    pidc->last_output = output;

    return output;
};

float _absolute_control(PID_Controller *pidc, float cur_value, float target_value, float other)
{
    float output;
    pidc->pre_error[pidc->cur_error_offset] = target_value - cur_value;

    pidc->integral += pidc->pre_error[pidc->cur_error_offset];

    if(pidc->integral < pidc->config.min_integral)
        pidc->integral = pidc->config.min_integral;
    else if(pidc->integral > pidc->config.max_integral)
        pidc->integral = pidc->config.max_integral;

    output = pidc->config.kp * pidc->pre_error[pidc->cur_error_offset]
             + pidc->config.ki * pidc->integral
             + pidc->config.kd *
               (pidc->pre_error[pidc->cur_error_offset]
                - pidc->pre_error[(pidc->cur_error_offset + 2) % 3])
             + pidc->config.ko * other;

    pidc->cur_error_offset = (uint8_t) ((pidc->cur_error_offset + 1) % 3);

    if(output > pidc->config.max_output)
        output = pidc->config.max_output;
    else if(output < pidc->config.min_output)
        output = pidc->config.min_output;

    return output;
}

float PID_Controller_Calc(PID_Controller *pidc, float cur_value, float target_value, float other, float *incre_output)
{
    if(pidc->config.mode == PID_Controller_Mode_Incremental)
    {
        _incremental_control(pidc, cur_value, target_value, other, incre_output);
    }
    else
    {
        _absolute_control(pidc, cur_value, target_value, other);
    }
}