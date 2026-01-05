#include "LowPassFilter.h"

LowPassFilter speedfilter;
LowPassFilter current_q_filter;

/**
 * @brief 初始化低通滤波器
 * @param filter: 滤波器结构体指针
 * @param time_constant: 时间常数，单位秒
 * - 较小的Tf值：响应快，滤波效果弱
 * - 较大的Tf值：响应慢，滤波效果好
**/

void LowPassFilter_Init(LowPassFilter* filter, float time_constant)
{
 
    // 设置时间常数 
    filter->Tf = time_constant;
    
    // 初始化上一次输出值为0 
    filter->y_prev = 0.0f;
    
    // 获取当前系统时间作为初始时间戳
    filter->timestamp_prev = HAL_GetTick();

}

/**
 * @brief 更新滤波器并获取滤波后的值
 * @param filter: 滤波器结构体指针
 * @param x: 新的输入值
 * @return 滤波后的输出值
**/
float LowPassFilter_Update(LowPassFilter* filter, float x)
{
    // 获取当前时间戳
    unsigned long timestamp = HAL_GetTick();
    
    // 计算时间间隔（毫秒转换为秒）
    float dt = (timestamp - filter->timestamp_prev) /1000.0f;
    
    // 处理异常时间间隔
    if (dt < 0.0f || dt < 1e-4) {
        // 时间间隔太小，直接返回上次值
        return filter->y_prev;
    }  
		
		else if (dt > 0.3f) 
		{
        // 时间间隔过长（超过300ms），重置滤波器状态
        filter->y_prev = x;
        filter->timestamp_prev = timestamp;
        return x;
    }
    
    // 计算滤波器系数alpha
    float alpha = filter->Tf / (filter->Tf + dt);
    
    // 应用低通滤波公式：
    // 新输出 = alpha * 旧输出 + (1-alpha) * 新输入
    float y = alpha * filter->y_prev + (1.0f - alpha) * x;
    
    // 更新滤波器状态
    filter->y_prev = y;           // 保存当前输出供下次使用
    filter->timestamp_prev = timestamp; // 更新时间戳
    
    return y;
}

