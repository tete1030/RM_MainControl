//
// Created by Texot Qexoq on 5/18/15.
//

#ifndef __PID_H__
#define __PID_H__

typedef enum PID_Controller_Mode
{
    PID_Controller_Mode_Incremental = 0,
    PID_Controller_Mode_Absolute
} PID_Controller_Mode;

typedef struct PID_Controller_Configuration
{
    PID_Controller_Mode mode;
    float kp;
    float ki;
    float kd;
    float ko;
    float max_output;
    float min_output;
    float max_integral;
    float min_integral;
} PID_Controller_Configuration;

typedef struct PID_Controller
{
    PID_Controller_Configuration config;

    float sum_kp_ki_kd;
    float sum_kp_2kd;

    float pre_error[3];
    float integral;
    uint8_t cur_error_offset;
    float last_output;
} PID_Controller;

void PID_Controller_Init(PID_Controller *pidc, PID_Controller_Configuration *pci);
float PID_Controller_Calc(PID_Controller *pidc, float cur_value, float target_value, float other, float *incre_output);


#endif //__PID_H__
