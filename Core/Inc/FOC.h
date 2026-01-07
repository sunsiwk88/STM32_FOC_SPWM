#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"

typedef enum {
    FOC_MODE_IDLE,      // 空闲/停止
    FOC_MODE_CURRENT,   // 电流环 (力矩)
    FOC_MODE_VELOCITY,  // 速度环
    FOC_MODE_ANGLE      // 角度环
} FOC_ControlMode;
extern FOC_ControlMode ctrl_mode;
	
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



/**
 * @brief  电流闭环控制
 * @param  target_current: 目标电流 (A)
 */
void set_Foc_current(float target_current);

#endif


