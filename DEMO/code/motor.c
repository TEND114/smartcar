#include "zf_common_headfile.h"
#include "my_common.h"

motor1 motor_l;
motor1 motor_r;

extern int start;
extern uint8_t key1_pressed_event;
extern uint8_t key1_processed;

extern int16 encoder_data_1;
extern int16 encoder_data_2; 
extern int16 encoder_data_3;
extern int16 encoder_data_4;
extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];
extern float adc_error_filtered;

extern PID motor_pid_l;
extern PID motor_pid_r;

// 计算实际速度
// 速度计算参数
#define ENCODER_PPR         1024.0f    // 编码器线数
#define GEAR_RATIO          1.0f       // 减速比
#define WHEEL_DIAMETER      0.064f     // 车轮直径(m)
#define CONTROL_FREQ        100.0f     // 控制频率(Hz)
#define SAMPLE_TIME         (1.0f/CONTROL_FREQ) // 采样时间(s)

// 计算常数
#define ENCODER_PULSES_PER_REV (ENCODER_PPR * 2.0f) // 每转脉冲数(四倍频)
#define RPM_TO_PULSE_PER_S (ENCODER_PULSES_PER_REV / 60.0f) // RPM转脉冲/秒
#define PULSE_TO_RPM (60.0f / ENCODER_PULSES_PER_REV)       // 脉冲/秒转RPM

// 速度限制

#define MAX_SPEED_MPS       2.5f       // 最大线速度(m/s)
#define MIN_SPEED_RPM       50.0f      // 最小转速(死区)

#define DIFF_GAIN 1.0f // 差速增益系数

float max(float a, float b){
	return (a>b ? a : b);
}

float Limit(float a, float b, float c)
{
    if(b < a)
        return a;
    else if(b > c)
        return c;
    else
        return b;
}

void motor_init(motor1 *motor)
{
    motor->target_speed = 0;
		motor->current_speed = 0;
    motor->duty = 0;
    motor->encoder_raw = 0;
    motor->total_encoder = 0;
		motor->speed_rpm = 0;
		motor->speed_mps = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用编码器值修改电机参数，实现闭环控制
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     需要先初始化编码器并获取encoder_data_1,encoder_data_2
//-------------------------------------------------------------------------------------------------------------------

//void motor_use_encoder(void){
  //  motor_l.encoder_raw = encoder_data_1;
    //motor_l.encoder_speed = (encoder_data_1) * 0.8 + motor_l.encoder_speed * 0.2;
    //motor_l.total_encoder += motor_l.encoder_speed * SAMPLE_TIME; // 知道车走的总路程
    //encoder_clear_count(ENCODER_1);
    
   // motor_r.encoder_raw = encoder_data_2;
    //motor_r.encoder_speed = (-encoder_data_2) * 0.8 + motor_r.encoder_speed * 0.2;
    //motor_r.total_encoder += motor_r.encoder_speed * SAMPLE_TIME; // 知道车走的总路程
    //encoder_clear_count(ENCODER_2);
//}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     精确计算电机速度
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     使用物理单位，提高鲁棒性
//-------------------------------------------------------------------------------------------------------------------
void motor_speed_calculate(void)
{
    
    
    // 读取编码器值 (注意：这里需要根据你的编码器接口调整)
    int32_t encoder_l_current = encoder_data_1;
    int32_t encoder_r_current = -encoder_data_2; // 实际情况是这样的
    
    // 计算脉冲增量 (处理溢出)
    int32_t delta_pulse_l = encoder_l_current;
    int32_t delta_pulse_r = encoder_r_current;
    
    // 更新编码器记录
    motor_l.encoder_last = encoder_l_current;
    motor_r.encoder_last = encoder_r_current;
    motor_l.total_encoder += delta_pulse_l;
    motor_r.total_encoder += delta_pulse_r;
    
    // 计算转速 (RPM)
    // 转速 = (脉冲增量 / 采样时间) / (每转脉冲数) * 60
    float speed_pulse_per_s_l = delta_pulse_l / 0.01f; // 脉冲/秒
    float speed_pulse_per_s_r = delta_pulse_r / 0.01f; // 脉冲/秒
    
    motor_l.speed_rpm = speed_pulse_per_s_l * PULSE_TO_RPM;
    motor_r.speed_rpm = speed_pulse_per_s_r * PULSE_TO_RPM;
    
    // 计算线速度 (m/s)
    // 线速度 = 转速 * 车轮周长 / 60
    float wheel_circumference = WHEEL_DIAMETER * PI;
    motor_l.speed_mps = motor_l.speed_rpm * wheel_circumference / 60.0f;
    motor_r.speed_mps = motor_r.speed_rpm * wheel_circumference / 60.0f;
    
    // 低通滤波
    motor_l.current_speed = motor_l.speed_rpm*0.5+motor_l.current_speed*0.5; // 可以直接使用，或添加滤波
    motor_r.current_speed = motor_r.speed_rpm*0.5+motor_r.current_speed*0.5;
    
    // 清除编码器计数 (如果硬件支持)
    encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_2);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动控制
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------

void car_start(void){
		
    // 按键检测逻辑
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS) {
        if(!key1_processed) {
            key1_pressed_event = 1;
            key1_processed = 1;
        }
    } else {
        key1_processed = 0;  // 按键释放时重置处理标志
    }
    
    // 处理按键事件
    if(key1_pressed_event) {
        key1_pressed_event = 0;  // 清除事件
        start++;
    }
    
    // 电机控制逻辑
    if(start != 0){
        if(start % 2 == 1){
            Motor_Final_Control();
					//Motor_Control(STRAIGHT_SPEED_RPM,STRAIGHT_SPEED_RPM);
					//Motor_Control(CURVE_SPEED_RPM,CURVE_SPEED_RPM);
					//Motor_Control(MIN_CURVE_RPM,MIN_CURVE_RPM);
        } else {
            Motor_Control(0.0, 0.0);
        }
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停车控制
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------

void car_stop(void){};


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度环PID控制
// 参数说明     speed_l, speed_r: 目标转速 (RPM)
// 返回参数     无
// 备注信息     使用位置式PID，物理意义明确
//-------------------------------------------------------------------------------------------------------------------
void Motor_Control(float target_speed_l, float target_speed_r)
{
    // 设置目标速度 (限制范围)
    motor_l.target_speed = Limit(-MAX_SPEED_RPM, target_speed_l, MAX_SPEED_RPM);
    motor_r.target_speed = Limit(-MAX_SPEED_RPM, target_speed_r, MAX_SPEED_RPM);
    
    // 死区处理
    if(fabsf(motor_l.target_speed) < MIN_SPEED_RPM) {
        motor_l.target_speed = 0;
    }
    if(fabsf(motor_r.target_speed) < MIN_SPEED_RPM) {
        motor_r.target_speed = 0;
    }
    
    float new_duty_l = motor_l.duty + PID_Increase(&motor_pid_l, motor_l.current_speed, motor_l.target_speed);
    float new_duty_r = motor_r.duty + PID_Increase(&motor_pid_r, motor_r.current_speed, motor_r.target_speed);
    
    // 输出限幅
    new_duty_l = Limit(-100.0f, new_duty_l, 100.0f);
    new_duty_r = Limit(-100.0f, new_duty_r, 100.0f);
    
    // 更新电机输出
    motor_l.duty = new_duty_l;
    motor_r.duty = new_duty_r;
    
    // 设置电机方向和控制信号
    if(motor_l.duty >= 0) {
        gpio_set_level(MOTOR1_DIR, GPIO_HIGH);  // 正转
        pwm_set_duty(MOTOR1_PWM, motor_l.duty * (PWM_DUTY_MAX / 100.0f));
    } else {
        gpio_set_level(MOTOR1_DIR, GPIO_LOW);   // 反转
        pwm_set_duty(MOTOR1_PWM, (-motor_l.duty) * (PWM_DUTY_MAX / 100.0f));
    }
    
    if(motor_r.duty >= 0) {
        gpio_set_level(MOTOR2_DIR, GPIO_HIGH);  // 正转
        pwm_set_duty(MOTOR2_PWM, motor_r.duty * (PWM_DUTY_MAX / 100.0f));
    } else {
        gpio_set_level(MOTOR2_DIR, GPIO_LOW);   // 反转
        pwm_set_duty(MOTOR2_PWM, (-motor_r.duty) * (PWM_DUTY_MAX / 100.0f));
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算弯道差速值 (基于速度)
// 参数说明     error: 电感偏差值
// 返回参数     差速值 (RPM)
//-------------------------------------------------------------------------------------------------------------------
float calculate_curve_diff_rpm(float error)
{
    float base_speed = CURVE_SPEED_RPM; // 需要重新定义，如 250.0f
    
    // 差速计算
    float speed_diff = DIFF_GAIN * error * base_speed;
    
    // 固定范围限幅，避免依赖当前duty
    float max_diff = base_speed * 0.05f; // 最大差速为基础速度的5%
    speed_diff = Limit(-max_diff, speed_diff, max_diff);
    
    return speed_diff;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     总体电机控制
// 参数说明     无
// 返回参数     无
//-------------------------------------------------------------------------------------------------------------------
void Motor_Final_Control(void)
{
    
    if(fabsf(adc_error_filtered) < 0.1f) { 
        // 直道：左右轮相同速度
        Motor_Control(500, 500);
    }
    else {
        // 弯道：差速控制
        float speed_diff = calculate_curve_diff_rpm(adc_error_filtered);
        float base_speed = 300;
        
        // 应用差速
        // error > 0: 车偏左，需要右转 -> 左轮减速，右轮加速
        float left_speed = base_speed - speed_diff;
        float right_speed = base_speed + speed_diff;
        
        // 限幅保护
        left_speed = Limit(MIN_CURVE_RPM, left_speed, STRAIGHT_SPEED_RPM);
        right_speed = Limit(MIN_CURVE_RPM, right_speed, STRAIGHT_SPEED_RPM);
        
        Motor_Control(left_speed, right_speed);
    }
}