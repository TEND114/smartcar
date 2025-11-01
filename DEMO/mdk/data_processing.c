#include "zf_common_headfile.h"
#include "my_common.h"
#include "data_processing.h"

// 声明外部变量
extern int16 encoder_data_1;
extern int16 encoder_data_2; 
extern int16 encoder_data_3;
extern int16 encoder_data_4;
extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];
// 移除了未使用的 adc_error

// 全局变量定义
float adc_error_filtered = 0;
float encoder_speed_left = 0;
float encoder_speed_right = 0;

// 滤波器实例定义
moving_average_filter_t adc_filter[ADC_CHANNEL_NUMBER];

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     数据处理初始化
// 参数说明     无
// 返回参数     无
//-------------------------------------------------------------------------------------------------------------------
void data_processing_init(void)
{
    // 初始化ADC通道滤波器
    for(int i = 0; i < ADC_CHANNEL_NUMBER; i++) {
        for(int j = 0; j < 8; j++) {
            adc_filter[i].buffer[j] = 0;
        }
        adc_filter[i].index = 0;
        adc_filter[i].sum = 0;
        adc_filter[i].count = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     滑动平均滤波
// 参数说明     filter: 滤波器指针
//             new_value: 新采样值
// 返回参数     滤波后的值
//-------------------------------------------------------------------------------------------------------------------
static uint16_t moving_average_filter(moving_average_filter_t* filter, uint16_t new_value)
{
    if(filter->count >= 8) {
        filter->sum -= filter->buffer[filter->index];
    }
    
    filter->buffer[filter->index] = new_value;
    filter->sum += new_value;
    filter->index = (filter->index + 1) % 8;
    
    if(filter->count < 8) {
        filter->count++;
    }
    
    return (uint16_t)(filter->sum / filter->count);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显式归一化处理
// 参数说明     left_val: 左电感原始值
//             right_val: 右电感原始值  
//             left_norm: 归一化后的左电感值（输出参数）
//             right_norm: 归一化后的右电感值（输出参数）
// 返回参数     无
//-------------------------------------------------------------------------------------------------------------------
static void normalize_adc_values(uint16_t left_val, uint16_t right_val, float* left_norm, float* right_norm)
{
    uint32_t total = (uint32_t)left_val + (uint32_t)right_val;
    
    if(total == 0) {
        *left_norm = 0.5f;
        *right_norm = 0.5f;
    } else {
        *left_norm = (float)left_val / (float)total;
        *right_norm = (float)right_val / (float)total;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC数据处理
// 参数说明     无
// 返回参数     无
//-------------------------------------------------------------------------------------------------------------------
void process_adc_data(void)
{
    // 1. 先滤波
    uint16_t adc_left_filtered = moving_average_filter(&adc_filter[0], adc_buffer[0]);
    uint16_t adc_right_filtered = moving_average_filter(&adc_filter[3], adc_buffer[3]);
    
    // 2. 再显式归一化
    float left_normalized, right_normalized;
    normalize_adc_values(adc_left_filtered, adc_right_filtered, &left_normalized, &right_normalized);
    
    // 3. 最后基于归一化值计算误差
    adc_error_filtered = left_normalized - right_normalized;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器数据处理
// 参数说明     无
// 返回参数     无
//-------------------------------------------------------------------------------------------------------------------
void process_encoder_data(void)
{
    encoder_speed_left = (float)encoder_data_1;
    encoder_speed_right = (float)encoder_data_2;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取归一化值（用于调试）
// 参数说明     left_norm: 左电感归一化值（输出参数）
//             right_norm: 右电感归一化值（输出参数）
// 返回参数     无
//-------------------------------------------------------------------------------------------------------------------
void get_normalized_values(float* left_norm, float* right_norm)
{
    uint16_t adc_left_filtered = moving_average_filter(&adc_filter[0], adc_buffer[0]);
    uint16_t adc_right_filtered = moving_average_filter(&adc_filter[3], adc_buffer[3]);
    normalize_adc_values(adc_left_filtered, adc_right_filtered, left_norm, right_norm);
}