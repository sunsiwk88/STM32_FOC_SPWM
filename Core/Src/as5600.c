#include "as5600.h"

#define abs(x) ((x) > 0 ? (x) : -(x))
#define _2PI 6.283185307179586f
#define _PI 3.141592653589793f

AS5600_T AngleSensor;

// 写入数据
HAL_StatusTypeDef _WriteData(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef sta = HAL_ERROR;
  sta = HAL_I2C_Master_Transmit(hi2c, dev_addr, data, size, HAL_MAX_DELAY);
  return sta;
}

// 读取数据
HAL_StatusTypeDef _ReadData(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef sta = HAL_ERROR;
  sta = HAL_I2C_Master_Receive(hi2c, (dev_addr | 1), data, size, HAL_MAX_DELAY);
  return sta;
}

// 初始化AS5600
int AS5600_Init(AS5600_T *a, I2C_HandleTypeDef *hi2c) {
    a->i2c_ins = hi2c;
    a->prev_angle_raw = AS5600_GetRawAngle(a); // 初始化原始角度值
    a->prev_update_ts = HAL_GetTick();         // 初始化时间戳
    a->rotation_offset = 0.0f;
    a->total_angle_rad = 0.0f;
    a->velocity_rad_s  = 0.0f;
    return 0;
}

// 获取原始角度值
uint16_t AS5600_GetRawAngle(AS5600_T *a) {
  uint16_t raw_angle;
  uint8_t buffer[2];
  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;

  _WriteData(a->i2c_ins, AS5600_ADDR, &raw_angle_register, 1);
  _ReadData(a->i2c_ins, AS5600_ADDR, buffer, 2);
  raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  return raw_angle;
}

/**
 * @brief 获取实时角度值 (单位: rad)
 * @note 执行I2C通信，获取原始角度值
 */
float AS5600_GetOnceAngle(AS5600_T *a) {
  return (float)(AS5600_GetRawAngle(a) / AS5600_RESOLUTION) * _2PI;
}

/**
* @brief 获取累计角度值 (单位: rad)
 * @note 执行I2C通信，获取原始角度值
 */
float AS5600_GetAngle(AS5600_T *a) {
  return (float)(a->prev_angle_raw / AS5600_RESOLUTION) * _2PI;
}

/**
 * @brief 获取速度 (单位: rad/s)
 * @note 执行I2C通信，直接调用 AS5600_Update 获取值
 */
float AS5600_GetVelocity(AS5600_T *a) {
    return a->velocity_rad_s;
}

/**
 * @brief 获取累计角度值 (单位: rad)
 * @note 执行I2C通信，直接调用 AS5600_Update 获取值
 */
float AS5600_GetAccumulateAngle(AS5600_T *a) {
    return a->total_angle_rad;
}

/**
 * @brief 更新AS5600并获取累计角度和速度
 * 使用该函数执行I2C通信，获取累计角度和速度
 */
void AS5600_Update(AS5600_T *a) {
    // 获取当前时间戳
    float current_angle_raw = AS5600_GetRawAngle(a);
    unsigned long current_ts = HAL_GetTick();

    // 计算时间差 (单位: s)
    float delta_t_s = (current_ts - a->prev_update_ts) / 1000.0f;
    
    // 时间差太小，不进行计算，避免计算错误
    if (delta_t_s < 5e-5f) {
        return;
    }

    // 使用状态机，计算上一个累计角度
    float prev_total_angle_rad = a->rotation_offset + (a->prev_angle_raw / AS5600_RESOLUTION) * _2PI;

    // 计算当前角度与上一个角度之差
    float delta_angle_raw = current_angle_raw - a->prev_angle_raw;
    if (abs(delta_angle_raw) > (0.8f * AS5600_RESOLUTION)) {
        // 计算累计角度偏移
        a->rotation_offset += (delta_angle_raw > 0 ? -_2PI : _2PI);
    }

    // 计算当前累计角度
    a->total_angle_rad = a->rotation_offset + (current_angle_raw / AS5600_RESOLUTION) * _2PI;

    // 计算角度变化
    float delta_angle_rad = a->total_angle_rad - prev_total_angle_rad;

   // 计算速度
    float raw_velocity = delta_angle_rad / delta_t_s;
		//低通滤波
    a->velocity_rad_s = LowPassFilter_Update(&speedfilter, raw_velocity);
		
    // 状态机，将当前角度值作为下一个周期的初始值
    a->prev_angle_raw = current_angle_raw;
    a->prev_update_ts = current_ts;
}

