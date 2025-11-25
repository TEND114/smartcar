#include "zf_common_headfile.h"
#include "my_common.h"

//定义存放编码器数值的变量
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;

//定义电磁信号adc相关变量
uint8 channel_index = 0;
adc_channel_enum channel_list[ADC_CHANNEL_NUMBER] = 
{
    ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4,
};
uint16_t adc_buffer[ADC_CHANNEL_NUMBER];

//根据电感数值计算得到的误差值（不再使用，使用滤波后的误差adc_error_filtered）
float adc_error;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取编码器数值
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     编码器数值先读取，后立即清零
//				此函数一次性读取了4个编码器数值
//-------------------------------------------------------------------------------------------------------------------
void get_encoder()
{
	encoder_data_1 = encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

    encoder_data_2 = encoder_get_count(ENCODER_2);                              // 获取编码器计数
    encoder_clear_count(ENCODER_2);                                             // 清空编码器计数
    
    encoder_data_3 = encoder_get_count(ENCODER_3);                              // 获取编码器计数
    encoder_clear_count(ENCODER_3);                                             // 清空编码器计数

    encoder_data_4 = encoder_get_count(ENCODER_4);                              // 获取编码器计数
    encoder_clear_count(ENCODER_4);                                             // 清空编码器计数
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取ADC数值
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     读取4路ADC得到四路电感采集数值
//				此处误差值error直接由第一路和第四路电感直接做差得到，其余误差计算算法可自行尝试
//-------------------------------------------------------------------------------------------------------------------
void get_adc()
{
	  for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
  {
			adc_buffer[channel_index] = adc_convert(channel_list[channel_index]);   //采集电磁adc信号，若需滤波，自行调用或自己写滤波函数
  }
	adc_error = (float)(adc_buffer[0] - adc_buffer[3]);//用电磁信号计算偏差，两边电感采样的adc值相减
}





void get_data()
{
	get_encoder();
	get_adc();
	
	//process_encoder_data(); // 转换为速度值
  process_adc_data();     // 滤波 + 归一化 + 误差计算
	
}