#include "zf_common_headfile.h"
#include "my_common.h"

//定义舵机打角值
float servo_motor_angle = SERVO_MOTOR_M; 
//PID系数kp
float kp = 0.008;
extern float adc_error_filtered;

extern float left_normalized;
extern float right_normalized;

extern PID servo_pid;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置舵机PWM
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     舵机打角值在中值基础上加上电感误差值乘以kp系数得到
//				打角值做限幅处理后直接输出舵机执行，根据实际接线情况选择三个舵机中的一个舵机即可
//-------------------------------------------------------------------------------------------------------------------
void set_servo_pwm()
{
	float pid_output = PID_Servo_Control(&servo_pid, adc_error_filtered, 0.0f, 1000.0f);
	
    
    // 计算舵机角度
  servo_motor_angle = SERVO_MOTOR_M - pid_output;
	//打角限幅
	if(servo_motor_angle > SERVO_MOTOR_R_MAX)servo_motor_angle = SERVO_MOTOR_R_MAX;
	if(servo_motor_angle < SERVO_MOTOR_L_MAX)servo_motor_angle = SERVO_MOTOR_L_MAX;

	//设置舵机打角
	pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}

// 实现对赛道元素的识别和控制舵机的选择

void Servo_Final_Control(){
	
}