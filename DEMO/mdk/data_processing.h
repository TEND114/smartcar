#ifndef __DATA_PROCESSING_H__
#define __DATA_PROCESSING_H__

#include "zf_common_headfile.h"
#include "my_common.h"

// 滤波器结构体定义
typedef struct {
    uint16_t buffer[8];  // 滑动窗口
    uint8_t index;       // 当前索引
    uint32_t sum;        // 窗口内数据和
    uint8_t count;       // 有效数据计数
} moving_average_filter_t;

// 声明外部变量（在get_data.c中定义）
extern int16 encoder_data_1;
extern int16 encoder_data_2;
extern int16 encoder_data_3; 
extern int16 encoder_data_4;
extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];
extern float adc_error;

// 声明全局变量（在data_processing.c中定义）
extern float adc_error_filtered;
extern float encoder_speed_left;
extern float encoder_speed_right;

// 函数声明
void data_processing_init(void);
void process_adc_data(void);
void process_encoder_data(void);
float calculate_enhanced_error(void);

#endif