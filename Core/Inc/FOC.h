#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"


extern int PP,DIR;

extern float zero_electric_angle ;
extern float voltage_power_supply;

extern float	Ualpha, Ubeta ;
extern float Ua , Ub , Uc ;
extern uint32_t dc_a , dc_b , dc_c;

extern float I_q;
extern float Target_Iq;

extern volatile float serial_motor_target;

float _normalizeAngle(float angle);
float _electricalAngle(void);


//void FOC_Init(float power_supply);
//void FOC_AS5600_Init(int _PP,int _DIR);
void FOC_Init_Simple(float power_supply, int _PP, int _DIR);
void Pwm_Init(void);

void setPwm(float Ua, float Ub, float Uc);
void setTorque(float Uq,float Ud,float angle_el);

void set_Foc_angle(float target_angle);
void set_Foc_speed(float target_vel);

void set_Foc_current(float target_current);

//// 2. 速度-电流 双闭环 (速度环输出目标电流 -> 电流环)
//void set_Foc_velocity_current(float target_vel);

//// 3. 位置-速度-电流 三闭环 (位置环输出目标速度 -> 速度环输出目标电流 -> 电流环)
//void set_Foc_angle_velocity_current(float target_angle);

//// 4. 位置-电流 力位闭环 (位置环直接输出目标电流，类似弹簧)
//void set_Foc_angle_force(float target_angle);
// === 核心控制模式 (带限幅功能) ===

/**
 * @brief  电流闭环控制
 * @param  target_current: 目标电流 (A)
 */
void set_Foc_current(float target_current);

/**
 * @brief  速度-电流 双闭环控制 (带电流限制)
 * @param  target_velocity: 目标速度 (rad/s)
 * @param  limit_current:   最大允许电流 (A)
 */
void set_Foc_velocity_with_limit(float target_velocity, float limit_current);

/**
 * @brief  位置-速度-电流 三闭环控制 (带速度和电流限制)
 * @param  target_angle:    目标角度 (rad)
 * @param  limit_velocity:  最大允许速度 (rad/s)
 * @param  limit_current:   最大允许电流 (A)
 */
void set_Foc_angle_with_limits(float target_angle, float limit_velocity, float limit_current);

/**
 * @brief  位置-电流 力位控制/弹簧模式 (带电流限制)
 * @param  target_angle:   目标角度 (rad)
 * @param  limit_current:  最大允许电流 (A)
 */
void set_Foc_angle_force_with_limit(float target_angle, float limit_current);
#endif


