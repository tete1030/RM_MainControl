//
// Created by Texot Qexoq on 5/18/15.
//

#include <stdint.h>
#include "pid.h"

struct PID_Controller
{
    float kp;
    float ki;
    float kd;
    float ko;


    float pre_error[3];
    uint8_t cur_error_offset;
    float last_output;

    float max_value;
    float min_value;
};

void PID_Init(PID_Controller *pidc, float kp, float ki, float kd, float ko, float max_value, float min_value)
{
    pidc->kp = kp;
    pidc->ki = ki;
    pidc->kd = kd;
    pidc->ko = ko;

    pidc->pre_error[0] = 0;
    pidc->pre_error[1] = 0;
    pidc->pre_error[2] = 0;

    pidc->cur_error_offset = 0;
    pidc->last_output = 0;

    pidc->max_value = max_value;
    pidc->min_value = min_value;
}

float PID_Control(PID_Controller *pidc, float cur_value, float target_value, float other)
{
    float output;
    pidc->pre_error[pidc->cur_error_offset] = target_value - cur_value;

    output = pidc->kp * (pidc->pre_error[pidc->cur_error_offset] - pidc->pre_error[(pidc->cur_error_offset + 2) % 3])
             + pidc->ki * pidc->pre_error[pidc->cur_error_offset]
             + pidc->kd *
                        (pidc->pre_error[pidc->cur_error_offset]
                         - 2 * pidc->pre_error[(pidc->cur_error_offset + 2) % 3]
                         + pidc->pre_error[(pidc->cur_error_offset + 1) % 3])
             + pidc->ko * other;

    output += pidc->last_output;

    if(output > pidc->max_value)
        output = pidc->max_value;
    else if(output < pidc->min_value)
        output = pidc->min_value;

    pidc->last_output = output;

    return output;
};