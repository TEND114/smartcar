#include "my_common.h"
#include "zf_common_headfile.h"

PID servo_pid = PID_CREATE(20,0,5,0.5);// 500 + 300(RPM)

float Kp_base = 0.7;
float Kd_base = 5;

PID motor_pid_l = PID_CREATE(0.04,0.02,0.07,1);//200RPM
PID motor_pid_r = PID_CREATE(0.04,0.02,0.07,1);

//PID motor_pid_l = PID_CREATE(0.04,0.01,0.08,1);//300RPM
//PID motor_pid_r = PID_CREATE(0,0,0,1);

void dynamic_pid_value_set(void){
	if(motor_l.target_speed > 250 && motor_l.target_speed < 350){
				motor_pid_l.Kp = 0.04;
        motor_pid_l.Ki = 0.01;
        motor_pid_l.Kd = 0.08;
        motor_pid_l.LowPass = 1;// 300rpm 直道速度
				motor_pid_l.Out_I = 0;
        motor_pid_l.Out_D = 0;

	}
	if(motor_r.target_speed > 250 && motor_r.target_speed < 350){
				motor_pid_r.Kp = 0.04;
				motor_pid_r.Ki = 0.01;
        motor_pid_r.Kd = 0.08;
        motor_pid_r.LowPass = 1;// 300rpm 直道速度
				motor_pid_r.Out_I = 0;
        motor_pid_r.Out_D = 0;
	}
	if(motor_l.target_speed > 150 && motor_l.target_speed < 250){
				motor_pid_l.Kp = 0.04;
        motor_pid_l.Ki = 0.02;
        motor_pid_l.Kd = 0.07;
        motor_pid_l.LowPass = 1;// 200rpm 弯道速度
				motor_pid_l.Out_I = 0;
        motor_pid_l.Out_D = 0;

	}
	if(motor_r.target_speed > 150 && motor_r.target_speed < 250){
				motor_pid_r.Kp = 0.04;
				motor_pid_r.Ki = 0.02;
        motor_pid_r.Kd = 0.07;
        motor_pid_r.LowPass = 1;// 200rpm 弯道速度
				motor_pid_r.Out_I = 0;
        motor_pid_r.Out_D = 0;
	}
}
	
// 位置PID
float PID_Normal(PID *PID, float NowData, float Point)
{
    PID->Error = Point - NowData;
    PID->Out_D = (PID->Error - PID->Out_P) * PID->LowPass + PID->Out_D * (1 - PID->LowPass);
    PID->Out_P = PID->Error;
    return (PID->Kp * PID->Out_P + PID->Kd * PID->Out_D);
}	


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机PID控制
// 参数说明     pid: PID控制器指针
//             NowData: 当前RPM
//             Point: 目标RPM
//             integral_limit: 积分限幅
// 返回参数     PID输出值（duty增量）
//-------------------------------------------------------------------------------------------------------------------
float PID_Servo_Control(PID *pid, float NowData, float Point, float integral_limit)
{
    // 计算当前误差
    pid->Error = Point - NowData;
    
    // 比例项
    pid->Out_P = pid->Error;
    
    // 积分项（带限幅）
    pid->Out_I += pid->Error;
    if(pid->Out_I > integral_limit) pid->Out_I = integral_limit;
    if(pid->Out_I < -integral_limit) pid->Out_I = -integral_limit;
    
    // 微分项（带低通滤波）
    float derivative = pid->Error - pid->LastError;
    pid->Out_D = derivative * pid->LowPass + pid->Out_D * (1 - pid->LowPass);
    
    // 保存历史误差
    pid->PrevError = pid->LastError;
    pid->LastError = pid->Error;
    
    // 计算PID输出
    float output = pid->Kp * pid->Out_P + pid->Ki * pid->Out_I + pid->Kd * pid->Out_D;
    
    // 输出限幅
    if(output > 15) output = 15;
    if(output < -15) output = -15;
    
    return output;
}
	
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式PID计算函数
// 参数说明     pid PID结构体指针，NowData 当前值，Point  目标值
// 返回参数     PID计算出的增量值
// 使用示例     无
// 备注信息     DIR控制电机方向，duty控制为占空比控制速度，若车轮反转，尝试将对应的DIR设置为低
//-------------------------------------------------------------------------------------------------------------------	
	
float PID_Increase(PID *PID, float NowData, float Point)
{
		//dynamic_pid_value_set();
    PID->Error = Point - NowData;
    PID->Out_P = (PID->Error - PID->LastError);          // 比例增量
		float integral_gain = 1.0f;  // 默认全增益
    
    if(fabsf(PID->Error) > 60.0f) {  // 误差很大时（启动瞬间）
        integral_gain = 0.2f;  // 只使用30%的积分增益
    } else if(fabsf(PID->Error) > 100.0f) {
        integral_gain = 0.7f;  // 使用60%的积分增益
    }
    PID->Out_I = PID->Error * integral_gain;                             // 积分增量
    PID->Out_D = (PID->Error - 2 * PID->LastError + PID->PrevError); // 微分增量
    
    // 一阶低通滤波
    PID->PrevError = 0.8 * PID->LastError + 0.2 * PID->PrevError;
    PID->LastError = 0.8 * PID->Error + 0.2 * PID->LastError;
    PID->LastData = NowData;

    return (PID->Kp * PID->Out_P + PID->Ki * PID->Out_I + PID->Kd * PID->Out_D);
}