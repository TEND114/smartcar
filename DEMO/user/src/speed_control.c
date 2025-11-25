#include "zf_common_headfile.h"
#include "my_common.h"

//控制电机速度，可取0-100，若太大可调低，若使用3S电池，占空比应限幅60%，默认10%
int8 duty = 10;	

extern struct motor1 motor_l;
extern struct motor1 motor_r;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机PWM
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     DIR控制电机方向，duty控制为占空比控制速度，若车轮反转，尝试将对应的DIR设置为低
//				如若MOTOR1对应的电机反转，可以尝试调用gpio_set_level(MOTOR1_DIR, GPIO_LOW)，使得电机按理想方向旋转
//-------------------------------------------------------------------------------------------------------------------
void set_speed_pwm()
{
	//占空比限幅
	if(duty > MAX_PWM_DUTY)duty = MAX_PWM_DUTY;// 最大设定为50%
	
	gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                                   // DIR输出高电平
	pwm_set_duty(MOTOR1_PWM, duty * (PWM_DUTY_MAX / 100));                   // 计算占空比

	gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                                   // DIR输出高电平
	pwm_set_duty(MOTOR2_PWM, duty * (PWM_DUTY_MAX / 100));                   // 计算占空比

	gpio_set_level(MOTOR3_DIR, GPIO_HIGH);                                   // DIR输出高电平
	pwm_set_duty(MOTOR3_PWM, duty * (PWM_DUTY_MAX / 100));                   // 计算占空比

	gpio_set_level(MOTOR4_DIR, GPIO_HIGH);                                   // DIR输出高电平
	pwm_set_duty(MOTOR4_PWM, duty * (PWM_DUTY_MAX / 100));                   // 计算占空比
}