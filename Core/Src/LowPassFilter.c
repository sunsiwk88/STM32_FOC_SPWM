#include "LowPassFilter.h"

LowPassFilter speedfilter;
LowPassFilter current_q_filter;

/**
 * @brief 初始化低通滤波器
 * @param filter: 低通滤波器结构体指针
 * @param time_constant: 时间常数 单位：ms
 * - 小Tf值对应响应快，滤波效果差
 * - 大Tf值对应响应慢，滤波效果好
**/

void LowPassFilter_Init(LowPassFilter* filter, float time_constant)
{
 
    // 时间常数 
    filter->Tf = time_constant;
    
    // 初始化第一个值为0 
    filter->y_prev = 0.0f;
    
    // 获取当前系统时间作为初始时间戳
    filter->timestamp_prev = HAL_GetTick();

}

/**
 * @brief 更新低通滤波器并获取滤波后的值
 * @param filter: 低通滤波器结构体指针
 * @param x: 新的输入值
 * @return 滤波后的值
**/
float LowPassFilter_Update(LowPassFilter* filter, float x)
{
    // 获取当前时间戳
    unsigned long timestamp = HAL_GetTick();
    
    // 计算时间差
    float dt = (timestamp - filter->timestamp_prev) /1000.0f;
    
    // 处理异常时间差
    if (dt < 0.0f || dt < 1e-4) {
        // 时间差太小，直接返回上一个值
        return filter->y_prev;
    }  
		
		else if (dt > 0.3f) 
		{
        // 时间差太大，直接更新状态
        filter->y_prev = x;
        filter->timestamp_prev = timestamp;
        return x;
    }
    
    // 计算低通滤波系数alpha
    float alpha = filter->Tf / (filter->Tf + dt);
    
    // 应用低通滤波公式
    // y = alpha * y_prev + (1-alpha) * x
    float y = alpha * filter->y_prev + (1.0f - alpha) * x;
    
    // 更新状态
    filter->y_prev = y;           // 更新当前值
    filter->timestamp_prev = timestamp; // 更新时间戳
    
    return y;
}

