#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"

typedef enum {
    FOC_MODE_IDLE,      // 空闲模式
    FOC_MODE_CURRENT,   // 电流模式
    FOC_MODE_VELOCITY,  // 速度模式
    FOC_MODE_ANGLE      // 角度模式
} FOC_ControlMode;

extern FOC_ControlMode ctrl_mode;
// 电机相数
	
extern int PP,DIR; // 电机相数和方向

extern float zero_electric_angle ; // 零电角度
extern float voltage_power_supply; // 电源电压

extern float	Ualpha, Ubeta ; // α轴和β轴电压
extern float    Ua , Ub , Uc ; // 三相电压
extern uint32_t dc_a , dc_b , dc_c; // 三相占空比

extern float I_q; // Q轴电流
extern float Target_Iq; // 目标Q轴电流

extern volatile float serial_motor_target; // 串口目标值

float _normalizeAngle(float angle); // 归一化角度
float _electricalAngle(void); // 电角度 


//void FOC_Init(float power_supply);
//void FOC_AS5600_Init(int _PP,int _DIR);
void FOC_Init_Simple(float power_supply, int _PP, int _DIR);
void Pwm_Init(void);

void setPwm(float Ua, float Ub, float Uc);
void setTorque(float Uq,float Ud,float angle_el);

void set_Foc_angle(float target_angle);
void set_Foc_speed(float target_vel);

void set_Foc_current(float target_current);



/**
 * @brief  FOC控制电流
 * @param  target_current: 目标电流 (A)
 */
void set_Foc_current(float target_current);

#endif


