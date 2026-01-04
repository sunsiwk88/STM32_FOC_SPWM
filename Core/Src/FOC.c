#include "FOC.h"

// 初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define	PI 3.1415926f
#define _3PI_2 4.71238898038f

int PP=0,DIR=0;

float zero_electric_angle = 0;
float	voltage_power_supply= 0;

float	Ualpha, Ubeta = 0;
float Ua = 0, Ub = 0, Uc = 0;
uint32_t dc_a = 0, dc_b = 0, dc_c = 0;

volatile float serial_motor_target = 0;



// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
    float a = fmod(angle, 2*PI);
    return a >= 0 ? a : (a + 2*PI);
}

// 电角度求解
float _electricalAngle()
{
  return  _normalizeAngle((float)(DIR * PP) * AS5600_GetOnceAngle(&AngleSensor)-zero_electric_angle);
}

 // 初始化电机：
    //  power_supply: 供电电压
    //  _PP: 极对数
    //  _DIR: 方向(1 or -1)

void FOC_Init_Simple(float power_supply, int _PP, int _DIR)
{

    voltage_power_supply = power_supply;
    PP = _PP;
    DIR = _DIR;
    
    // 启动PWM
    Pwm_Init();
		// PID初始化
		PID_init();
		// 低通滤波器时间常数Tf设定为5ms
    LowPassFilter_Init(&speedfilter,0.005f);
    // 初始化传感器并等待稳定
    AS5600_Init(&AngleSensor, &hi2c1);
    HAL_Delay(100); 
    
    // 电机对齐
    setTorque(3,0,_3PI_2);
    HAL_Delay(500);
    
    // 读取零点
    float angle_sum = 0;
    for(int i = 0; i < 10; i++) 
		{
        AS5600_Update(&AngleSensor);
        angle_sum += _electricalAngle();
        HAL_Delay(10);
    }
    zero_electric_angle = angle_sum / 10.0f;
  
    // 释放力矩
    setTorque(0,0,_3PI_2);
    HAL_Delay(500);
	
    HAL_TIM_Base_Start_IT(&htim2);
   // printf("FOC初始化完成，零点：%.3f\r\n", zero_electric_angle);
}

void Pwm_Init()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc)
{
    // 先限制电压值
    Ua = _constrain(Ua, 0.0f, voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, voltage_power_supply); 
    Uc = _constrain(Uc, 0.0f, voltage_power_supply);
    
    // 再计算占空比
    dc_a = (uint32_t)((Ua / voltage_power_supply) * htim1.Instance->ARR);
    dc_b = (uint32_t)((Ub / voltage_power_supply) * htim1.Instance->ARR);
    dc_c = (uint32_t)((Uc / voltage_power_supply) * htim1.Instance->ARR);
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,dc_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,dc_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,dc_c);
}



 // 设置三相电压：
    //  Uq: Q轴电压，控制力矩
    //  Ud: D轴电压
    //  angle_el: 电角度

void setTorque(float Uq,float Ud,float angle_el) 
{
		Uq=_constrain(Uq,-voltage_power_supply/2,voltage_power_supply/2);
		
		angle_el = _normalizeAngle(angle_el);
	
    // 帕克逆变换
    Ualpha = -Uq * sin(angle_el); //简单情况下，默认Ud=0
    Ubeta = Uq * cos(angle_el); 

    // 克拉克逆变换
		//	加上voltage_power_supply/2是为了平移曲线到供电电压的中间，避免出现负数电压
    Ua = Ualpha + voltage_power_supply/2;
    Ub = (sqrt(3)*Ubeta - Ualpha)/2 + voltage_power_supply/2;
    Uc = (-Ualpha - sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
    
    setPwm(Ua, Ub, Uc);
}



//FOC闭环位置环 单位：弧度
void set_Foc_angle(float target_angle)
{
	
//	float Sensor_Angle=AS5600_GetAccumulateAngle(&AngleSensor);
//	float angle_out = Angle_Control(serial_motor_target - DIR * Sensor_Angle );
//	angle_out=_constrain(angle_out,-6,6);
//	setTorque( angle_out , _electricalAngle());
//	//printf("%.3f\r\n",Sensor_Angle);
		float current_angle = AS5600_GetAccumulateAngle(&AngleSensor);
	
    // 计算角度误差（弧度）
    float angle_error = target_angle - DIR * current_angle;
	
		angle_error = _constrain(angle_error, -3.0f, 3.0f);
	
    // 角度环PID → 目标速度
    float target_velocity = Angle_Control(angle_error);
	
		target_velocity = _constrain(target_velocity, -15.0f, 15.0f);

    // 读取当前速度
    float current_velocity = AS5600_GetVelocity(&AngleSensor);
    
    // 速度环PID → 力矩
    float vel_error = target_velocity - DIR * current_velocity;
		
    float torque_out = Speed_Control(vel_error);
    
    // 限制力矩
		torque_out = _constrain(torque_out, -8.0f, 8.0f);
    
    // 输出
    setTorque(torque_out,0,_electricalAngle());
	
}



//FOC闭环速度环 单位：弧度/秒
void set_Foc_speed(float target_vel)
{
	float current_velocity = AS5600_GetVelocity(&AngleSensor);
	float vel_error = target_vel - DIR * current_velocity ;
	float torque_out = Speed_Control(vel_error);
	// 限幅
	torque_out =_constrain(torque_out,-8,8);
	setTorque( torque_out,0,_electricalAngle());
}



