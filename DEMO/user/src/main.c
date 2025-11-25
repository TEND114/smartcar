/*********************************************************************************************************************
* 本例程基于RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）开发
* 
* 修改记录
* 日期              作者                备注
* 2025-09-30        wjm            first version
********************************************************************************************************************/

// *************************** 例程硬件连接说明 ***************************
/*
				连接好电机、主板、驱动板
				将电磁前瞻与运放、运放与主板连接好
			（如果有编码器，将编码器安装好并与主板连接）
			
*/
// *************************** 例程测试说明 ***************************
/*
			本例程包含功能：
			1.采集电磁信号并经过ADC传回MCU
			2.电磁信号直接做差计算偏差
			3.计算舵机打角
			4.设置电机pwm使电机旋转
			
			
			注意！注意！注意！注意！
			注意！注意！注意！注意！
			注意！注意！注意！注意！

			1.测试舵机最左，最右，中值，并在mycommon.h中修改，也可以拆下舵盘，先单独将舵机值设置为90.0，后再将舵盘安装上，则舵机对应值与本例程一致
			2.若担心电机旋转方向与与期望方向相反，可调换电机正负两根线在驱动板上的连接位置（注意不是电池！），也可以直接在speed_control.c中修改DIR的电平
			3.将小车正放在电磁线上，电磁线在小车中轴线上，调节运放的值，使得最左边与最右边电感值相等
			4.用电磁信号计算偏差时，最左电感值和最右电感值默认存放在adc_buffer[0]和adc_buffer[3]中，若接线不同请自行修改
			

*/

// **************************** 代码区域 ****************************

#include "zf_common_headfile.h"
#include "my_common.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完
extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];
extern int16 encoder_data_1;
extern int16 encoder_data_2;
extern float adc_error;
extern int8 duty;	
extern struct motor1 motor_l;
extern struct motor1 motor_r;

extern float left_normalized;
extern float right_normalized;

int start = 1;
uint8_t key1_pressed_event = 0;
uint8_t key1_processed = 0;

void Init_All(void);
void pid_handler(void);

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化

    system_delay_ms(300);           //等待主板其他外设上电完成
	
		
	Init_All();						//初始化所有外设
	
	
	pit_ms_init(PIT_CH, 10);//开启10ms中断

	
	
	
		//tft180_draw_line(20, 50, 10, 10,RGB565_RED);
	tft180_show_string(20, 0, "DianCi_demo");
	tft180_show_string(0, 15, "ADC_L:");
	//tft180_show_string(0, 30, "ADC1:");
	//tft180_show_string(0,30,"start:");
	//tft180_show_string(0, 45, "ADC4:");
	tft180_show_string(0, 60, "ADC_R:");
	tft180_show_string(0, 75, "ERROR");
	//tft180_show_string(0, 90, "ENCODER:");
	//tft180_show_string(0, 105, "DUTY:");
    while(1)
    {
        // 此处编写需要循环执行的代码
		//tft180_show_int(50, 15, adc_buffer[0], 4);
		//tft180_show_int(50, 30, adc_buffer[1], 4);
		//tft180_show_int(50, 45, adc_buffer[2], 4);
		//tft180_show_int(50, 60, adc_buffer[3], 4);
		
		 tft180_show_float(50, 15, left_normalized, 4, 4);	
		tft180_show_float(50, 60, right_normalized, 4, 4);		
		//tft180_show_int(50, 30, start, 4);
			
		tft180_show_float(50, 75, adc_error_filtered, 4, 4);
		
		//tft180_show_int(70, 90, encoder_data_1, 4);
		
		tft180_show_float(50, 105, PID_Servo_Control(&servo_pid, adc_error_filtered, 0.0f, 1000.0f), 4,4);
        
		//tft180_show_float(30,60,motor_l.speed_mps,4,4);
		//tft180_show_float(30,75, motor_r.speed_mps,4,4);
		//tft180_show_float(30,90,motor_l.target_speed,4,4);
		//tft180_show_float(30,105,motor_r.target_speed,4,4);


        // 此处编写需要循环执行的代码
		printf("%.1f,%.1f,%.1f,%.1f,%d\n", motor_l.target_speed, motor_l.current_speed, motor_r.target_speed, motor_r.current_speed,start);	
	}
}

//此初始化函数共初始化4路电机接口，4路编码器接口，4路电感ADC，以及3路舵机接口
void Init_All(void)
{
	//初始化电机驱动DRV8701
		gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR3_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR4_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
		
	//motor初始化
		motor_init(&motor_l);
    motor_init(&motor_r);
	
	//根据编码器类型和个数选择初始化函数
	//初始化编码器（正交）
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); // 初始化编码器模块与引脚 正交解码编码器模式
	
	//初始化编码器（步方向）
//    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚
//    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚
//    encoder_dir_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B); // 初始化编码器模块与引脚
//    encoder_dir_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); // 初始化编码器模块与引脚
	
	//初始化adc引脚
	adc_init(ADC_CHANNEL1, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL2, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL3, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL4, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
		
	//初始化舵机引脚
    pwm_init(SERVO_MOTOR1_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR2_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR3_PWM, SERVO_MOTOR_FREQ, 0);
		
	//初始化屏幕
	tft180_init();
	tft180_set_dir(TFT180_CROSSWISE);
}

extern void get_data();
extern void set_servo_pwm();
extern void set_speed_pwm();


//中断函数，在主函数中初始化为每10ms执行一次
void pit_handler (void)
{
	
	get_data();//获取传感器数据
	set_servo_pwm();//设置舵机打角
	motor_speed_calculate();
	key_scanner(); 
  car_start();
	
}