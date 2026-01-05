#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "main.h"


// 低通滤波器结构体定义
typedef struct {
    float Tf;               // 时间常数
    float y_prev;           // 上一次滤波输出值 
    unsigned long  timestamp_prev; // 上一次时间戳
} LowPassFilter;

extern LowPassFilter speedfilter;
extern LowPassFilter current_q_filter;

void LowPassFilter_Init(LowPassFilter* filter, float time_constant);
float LowPassFilter_Update(LowPassFilter* filter, float x);


#endif 

