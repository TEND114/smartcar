#ifndef __PID_H__
#define __PID_H__

#include "zf_common_headfile.h" 
#include "my_common.h"

typedef struct PID
{
    float Kp;
    float Ki;
    float Kd;
    float LowPass;

    float Out_P;
    float Out_I;
    float Out_D;
    
    float Error;
    float LastError;
    float PrevError;
    float LastData;
} PID;

#define PID_CREATE(_kp,_ki,_kd,_low_pass) \
{ \
    .Kp = _kp, \
    .Ki = _ki, \
    .Kd = _kd, \
    .LowPass = _low_pass, \
    .Out_P = 0, \
    .Out_I = 0, \
    .Out_D = 0, \
    .Error = 0, \
    .LastError = 0, \
    .PrevError = 0, \
    .LastData = 0 \
}

extern struct PID servo_pid;
extern struct PID motor_pid_l;
extern struct PID motor_pid_r;

float PID_Normal(PID *PID, float NowData, float Point);
float PID_Increase(PID *PID, float NowData, float Point);
float PID_Servo_Control(PID *pid, float NowData, float Point, float integral_limit);
void dynamic_pid_value_set(void);

#endif