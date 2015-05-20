//
// Created by Texot Qexoq on 5/18/15.
//

#ifndef __PID_H__
#define __PID_H__


struct PID_Controller;
typedef struct PID_Controller PID_Controller;

void PID_Init(PID_Controller *pidc, float kp, float ki, float kd, float ko, float max_value, float min_value);
float PID_Control(PID_Controller *pidc, float cur_value, float target_value, float other);


#endif //__PID_H__
