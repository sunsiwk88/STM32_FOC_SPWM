#ifndef __INLINE_CURRENT_H
#define __INLINE_CURRENT_H

#include "main.h"
#include "adc.h"

// ADC参考电压
#define ADC_REFERENCE_VOLTAGE   3.3f        // ADC参考电压
#define ADC_RESOLUTION_BITS     12          // ADC分辨率位数
#define ADC_MAX_VALUE          ((1 << ADC_RESOLUTION_BITS) - 1)  // 4095

// 电流传感器结构体
typedef struct {
    // 硬件参数
    float shunt_resistor;       // 分流电阻值 (Ω)
    float amp_gain;             // 放大倍数
    
    // 偏移值
    float offset_a;            // A相偏移电压
    float offset_b;            // B相偏移电压
    float offset_c;            // C相偏移电压
    
    // 转换系数
    float gain_a;               // A相电压转换系数
    float gain_b;               // B相电压转换系数
    float gain_c;               // C相电压转换系数
    
    // 电流值
    float current_a;            // A相电流 (A)
    float current_b;            // B相电流 (A)
    float current_c;            // C相电流 (A)
    
} InlineCurrent_T;

extern InlineCurrent_T CurrentSensor;

// 初始化电流传感器
void InlineCurrent_Init(InlineCurrent_T *curr, float shunt_resistor, float amp_gain);
// 校准电流传感器偏移值
void InlineCurrent_CalibrateOffsets(InlineCurrent_T *curr);
// 获取电流值
void InlineCurrent_GetPhaseCurrents(InlineCurrent_T *curr, float ia_voltage, float ib_voltage);
float InlineCurrent_ADCToVoltage(uint32_t adc_value);
void InlineCurrent_test(void);
#endif /* __INLINE_CURRENT_H */
