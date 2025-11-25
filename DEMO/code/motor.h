#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h" 
#include "my_common.h"

// 电机结构体
typedef struct motor1
{
	float target_speed;      // 目标速度 (单位：RPM 或 m/s)
    float current_speed;     // 当前速度 (单位：RPM 或 m/s)
    float duty;              // 输出PWM占空比
    int32_t encoder_raw;     // 编码器原始值
    int32_t encoder_last;    // 上一次编码器值
    int32_t total_encoder;   // 总编码器计数
    float speed_rpm;         // 转速 (RPM)
    float speed_mps;         // 线速度 (m/s)
}motor1;

extern struct motor1 motor_l;
extern struct motor1 motor_r;


// 函数声明
void motor_init(motor1 *motor);
void motor_speed_calculate(void);
void car_start(void);
void Motor_Control(float target_speed_l, float target_speed_r);
float calculate_curve_diff_rpm(float error);
void Motor_Final_Control(void);

#endif