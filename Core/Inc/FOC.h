#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"


extern int PP,DIR;

extern float zero_electric_angle ;
extern float voltage_power_supply;

extern float	Ualpha, Ubeta ;
extern float Ua , Ub , Uc ;
extern uint32_t dc_a , dc_b , dc_c;

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

#endif


