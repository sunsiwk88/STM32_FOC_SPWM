#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "main.h"


// 低通滤波器结构体定义
typedef struct {
    float Tf;               // 时间常数 (Time constant)
    float y_prev;           // 上一次滤波输出值 (Previous filter output)
    unsigned long  timestamp_prev; // 上一次时间戳 (Previous timestamp in milliseconds)
} LowPassFilter;

extern LowPassFilter speedfilter;
void LowPassFilter_Init(LowPassFilter* filter, float time_constant);
float LowPassFilter_Update(LowPassFilter* filter, float x);



#endif 

