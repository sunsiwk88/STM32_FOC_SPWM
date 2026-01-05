#ifndef __INLINE_CURRENT_H
#define __INLINE_CURRENT_H

#include "main.h"
#include "adc.h"

// ADC配置参数
#define ADC_REFERENCE_VOLTAGE   3.3f        // ADC参考电压
#define ADC_RESOLUTION_BITS     12          // ADC分辨率位数
#define ADC_MAX_VALUE          ((1 << ADC_RESOLUTION_BITS) - 1)  // 4095

// 电流传感器结构体
typedef struct {
    // 硬件参数
    float shunt_resistor;       // 分流电阻值 (Ω)
    float amp_gain;             // 运放增益
    
    // 校准偏移量
    float offset_a;            // A相零点偏移电压
    float offset_b;            // B相零点偏移电压
    float offset_c;            // C相零点偏移电压
    
    // 转换系数
    float gain_a;               // A相电压到电流转换系数
    float gain_b;               // B相电压到电流转换系数
    float gain_c;               // C相电流转换系数
    
    // 测量结果
    float current_a;            // A相电流 (A)
    float current_b;            // B相电流 (A)
    float current_c;            // C相电流 (A)
    
} InlineCurrent_T;

extern InlineCurrent_T CurrentSensor;

// 函数声明
void InlineCurrent_Init(InlineCurrent_T *curr, float shunt_resistor, float amp_gain);
void InlineCurrent_CalibrateOffsets(InlineCurrent_T *curr);
void InlineCurrent_GetPhaseCurrents(InlineCurrent_T *curr, float ia_voltage, float ib_voltage);
float InlineCurrent_ADCToVoltage(uint32_t adc_value);

#endif /* __INLINE_CURRENT_H */
