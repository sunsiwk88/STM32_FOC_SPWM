#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "main.h"


// 低通滤波器结构体
typedef struct {
    float Tf;               // 时间常数
    float y_prev;           // 上一个滤波值 
    unsigned long  timestamp_prev; // 上一个时间戳
} LowPassFilter;

extern LowPassFilter speedfilter;
extern LowPassFilter current_q_filter;

// 初始化低通滤波器
void LowPassFilter_Init(LowPassFilter* filter, float time_constant);
// 更新低通滤波器
float LowPassFilter_Update(LowPassFilter* filter, float x);


#endif 

