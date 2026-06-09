#include "InlineCurrent.h"

InlineCurrent_T CurrentSensor;

/**
 * @brief  初始化电流传感器
 * @param  curr: 电流传感器结构体指针
 * @param  shunt_resistor: 分流电阻值 
 * @param  amp_gain: 放大倍数
 * @retval None
 */
void InlineCurrent_Init(InlineCurrent_T *curr, float shunt_resistor, float amp_gain)
{
    curr->shunt_resistor = shunt_resistor;
    curr->amp_gain = amp_gain;
    
    // 电压转换为电流
    // I = U / (R_shunt * Gain)
    float volts_to_amps_ratio = 1.0f / shunt_resistor / (amp_gain/5);
    
    // 使用放大电路的增益
    curr->gain_a = 	-1*volts_to_amps_ratio;  // 使用放大电路的增益
    curr->gain_b = 	-1*volts_to_amps_ratio;
    curr->gain_c = 	volts_to_amps_ratio;
    
    // 初始化电流值
    curr->current_a = 0.0f;
    curr->current_b = 0.0f;
    curr->current_c = 0.0f;
    
    // 初始化偏移值
    curr->offset_a = 0.0f;
    curr->offset_b = 0.0f;
    curr->offset_c = 0.0f;
}

/**
 * @brief  ADC原始值转换为电压
 * @param  adc_value: ADC原始值 (0-4095)
 * @retval 转换后的电压值 (V)
 */
float InlineCurrent_ADCToVoltage(uint32_t adc_value)
{
    return ((float)ADC_REFERENCE_VOLTAGE * adc_value / ADC_MAX_VALUE ) - 1.65f;
}

/**
 * @brief 校准电流传感器偏移值
 * @note   在正常工作过程中使用
 * @param  curr: 电流传感器结构体指针
 * @retval None
 */
void InlineCurrent_CalibrateOffsets(InlineCurrent_T *curr)
{
    const uint16_t calibration_rounds = 1000;
    
    float sum_ua = 0.0f;
    float sum_ub = 0.0f;
//    float sum_uc = 0.0f;
    
    // 读取1000次ADC
    for (uint16_t i = 0; i < calibration_rounds; i++) 
		{
        // 等待ADC转换
        HAL_Delay(1);
        
        // 读取ADC注入转换完成后的值并转换为电压
        uint32_t adc1_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint32_t adc2_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        
        sum_ua += InlineCurrent_ADCToVoltage(adc1_value);
        sum_ub += InlineCurrent_ADCToVoltage(adc2_value);
    }
    
    // 计算平均值作为偏移值
    curr->offset_a = sum_ua / calibration_rounds;
    curr->offset_b = sum_ub / calibration_rounds;
   //curr->offset_uc = sum_uc / calibration_rounds;
}

/**
 * @brief  获取电流值
 * @param  curr: 电流传感器结构体指针
 * @param  ia_voltage: A相ADC电压 (V)
 * @param  ib_voltage: B相ADC电压 (V)
 * @retval None
 * @note   获取电流值 curr->current_a, curr->current_b, curr->current_c
 */
void InlineCurrent_GetPhaseCurrents(InlineCurrent_T *curr, float voltage_a, float voltage_b)
{
    // 去除偏移并应用增益得到电流值
    curr->current_a = (voltage_a - curr->offset_a) * curr->gain_a;
    curr->current_b = (voltage_b - curr->offset_b) * curr->gain_b;
    
    // 根据相电流关系：Ia + Ib + Ic = 0
    curr->current_c = -(curr->current_a + curr->current_b);
	
}

/**
 * @brief  测试电流传感器
 * @retval None
 */
void InlineCurrent_test()
{	
  setPwm(6, 0, 0);
  HAL_Delay(300);
  printf("%f,%f,%f\r\n", CurrentSensor.current_a, CurrentSensor.current_b, 
	-(CurrentSensor.current_a + CurrentSensor.current_b));

  setPwm(0, 6, 0);
  HAL_Delay(300);
   printf("%f,%f,%f\r\n", CurrentSensor.current_a, CurrentSensor.current_b, 
	-(CurrentSensor.current_a + CurrentSensor.current_b));

  setPwm(0, 0, 6);
  HAL_Delay(300);
  printf("%f,%f,%f\r\n", CurrentSensor.current_a, CurrentSensor.current_b, 
	-(CurrentSensor.current_a + CurrentSensor.current_b));
  setPwm(0, 0, 0);
}

