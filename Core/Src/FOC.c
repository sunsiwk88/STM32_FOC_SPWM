#include "FOC.h"
#include "InlineCurrent.h" 

FOC_ControlMode ctrl_mode=FOC_MODE_IDLE;

// 初始化约束
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define	PI 3.1415926f
#define _3PI_2 4.71238898038f
#define	_1_SQRT3 0.57735026919f
#define	_2_SQRT3 1.15470053838f
#define _SQRT3 1.732050807568877f


int PP=0,DIR=0;

float zero_electric_angle = 0;
float	voltage_power_supply= 0;

float	Ualpha, Ubeta = 0;
float Ua = 0, Ub = 0, Uc = 0;
uint32_t dc_a = 0, dc_b = 0, dc_c = 0;

volatile float serial_motor_target = 0;

float I_q = 0;
float Target_Iq=0;

// 一个角度的 [0,2PI]
float _normalizeAngle(float angle)
{
    float a = fmod(angle, 2*PI);
    return a >= 0 ? a : (a + 2*PI);
}

// 电角度
float _electricalAngle()
{
  return  _normalizeAngle((float)(DIR * PP) * AS5600_GetOnceAngle(&AngleSensor)-zero_electric_angle);
}


/**
 * @brief  初始化FOC
 * @param  power_supply: 电源电压
 * @param  _PP: 电机极对数
 * @param  _DIR: 方向(1 or -1)
 * @retval None
 */
void FOC_Init_Simple(float power_supply, int _PP, int _DIR)
{

    voltage_power_supply = power_supply;
    PP = _PP;
    DIR = _DIR;
    
    // 初始化PWM
    Pwm_Init();
		// PID初始化
		PID_init();
		// 低通滤波器时间常数Tf设置为5ms
    LowPassFilter_Init(&speedfilter,0.005f);
		// 电流低通滤波器时间常数Tf设置为5ms 
    LowPassFilter_Init(&current_q_filter, 0.005f);
    // 初始化角度传感器
    AS5600_Init(&AngleSensor, &hi2c1);
    HAL_Delay(100); 
    
    // 设置初始力矩
    setTorque(3,0,_3PI_2);
    HAL_Delay(100);
    
    // 获取角度
    float angle_sum = 0;
    for(int i = 0; i < 10; i++) 
		{
        AS5600_Update(&AngleSensor);
        angle_sum += _electricalAngle();
        HAL_Delay(10);
    }
    zero_electric_angle = angle_sum / 10.0f;
  
    // 释放扭矩
    setTorque(0,0,_3PI_2);
    HAL_Delay(100);
	
    HAL_TIM_Base_Start_IT(&htim2);
   // printf("FOC初始化完成，角度：%.3f\r\n", zero_electric_angle);
}

void Pwm_Init()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}


// 设置PWM
void setPwm(float Ua, float Ub, float Uc)
{
    // 限制电压值
    Ua = _constrain(Ua, 0.0f, voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, voltage_power_supply); 
    Uc = _constrain(Uc, 0.0f, voltage_power_supply);
    
    // 计算占空比
    dc_a = (uint32_t)((Ua / voltage_power_supply) * 2400);
    dc_b = (uint32_t)((Ub / voltage_power_supply) * 2400);
    dc_c = (uint32_t)((Uc / voltage_power_supply) * 2400);
    
	  dc_a = min(dc_a,2160); // 限制10%的死区时间
		dc_b = min(dc_b,2160);
		dc_c = min(dc_c,2160);
	
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,dc_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,dc_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,dc_c);
}

/**
 * @brief  设置扭矩
 * @param  Uq: Q轴电压
 * @param  Ud: D轴电压
 * @param  angle_el: 电角度
 * @retval None
 */
void setTorque(float Uq,float Ud,float angle_el) 
{
		Uq=_constrain(Uq,-voltage_power_supply/2,voltage_power_supply/2);
		Ud = _constrain(Ud, -voltage_power_supply/2, voltage_power_supply/2);
	
		angle_el = _normalizeAngle(angle_el);
	
    // Park变换
		Ualpha = -Uq * sin(angle_el) + Ud * cos(angle_el); 
    Ubeta  =  Uq * cos(angle_el) + Ud * sin(angle_el);

    // Clarke变换
		//	将voltage_power_supply/2作为中性点电压，限制在电压范围内
    Ua = Ualpha + voltage_power_supply/2;
    Ub = (_SQRT3*Ubeta - Ualpha)/2 + voltage_power_supply/2;
    Uc = (-Ualpha - _SQRT3*Ubeta)/2 + voltage_power_supply/2;
    
    setPwm(Ua, Ub, Uc);
}


/**
 * @brief  FOC速度控制
 * @param  target_vel:目标速度值 单位：rad/s
 * @retval None
 */
void set_Foc_speed(float target_vel)
{
	float current_velocity = AS5600_GetVelocity(&AngleSensor);	
	float vel_error = target_vel - DIR * current_velocity ;
	float torque_out = Speed_Control(vel_error);

	// 限制扭矩范围
	torque_out =_constrain(torque_out,-8,8);
	setTorque( torque_out,0,_electricalAngle());
//	 // 打印频率，影响性能
//    static uint16_t print_counter = 0;
//    if(++print_counter >= 10) {  // 每100ms打印一次
//        print_counter = 0;
//        printf("%.2f,%.2f\r\n", serial_motor_target, Sensor_Vel);
//    }
}

/**
 * @brief  FOC角度控制
 * @note   PID控制
 * @param  target_vel:目标角度值 单位：rad
 * @retval None
 */
void set_Foc_angle(float target_angle)
{
	
//	float Sensor_Angle=AS5600_GetAccumulateAngle(&AngleSensor);
//	float angle_out = Angle_Control(serial_motor_target - DIR * Sensor_Angle );
//	angle_out=_constrain(angle_out,-6,6);
//	setTorque( angle_out , _electricalAngle());
//	//printf("%.3f\r\n",Sensor_Angle);
		float current_angle = AS5600_GetAccumulateAngle(&AngleSensor);
	
    // 角度误差（积分）
    float angle_error = target_angle - DIR * current_angle;
		angle_error = _constrain(angle_error, -3.0f, 3.0f);
	
    // 角度PID 目标速度
    float target_velocity = Angle_Control(angle_error);
		target_velocity = _constrain(target_velocity, -15.0f, 15.0f);

    // 获取当前速度
    float current_velocity = AS5600_GetVelocity(&AngleSensor);
    
    // 速度PID 控制
    float vel_error = target_velocity - DIR * current_velocity;
    float torque_out = Speed_Control(vel_error);
    
    // 限制扭矩范围
		torque_out = _constrain(torque_out, -8.0f, 8.0f);
    
    // 设置扭矩
    setTorque(torque_out,0,_electricalAngle());
	
}

/**
 * @brief  FOC电流控制
 * @note   PID控制
 * @param  target_current: 目标电流值 单位：A
 * @retval None
 */
void set_Foc_current(float target_current)
{
		Target_Iq = target_current;
    // 获取电角度
    float angle_el = _electricalAngle();
    
    // 获取电流
    float Ia = CurrentSensor.current_a;
    float Ib = CurrentSensor.current_b;
    // float Ic = -(Ia + Ib); 
    
    // Clarke变换（Ia, Ib -> Ialpha, Ibeta）
    float I_alpha = Ia;
    float I_beta = _1_SQRT3 * Ia + _2_SQRT3 * Ib;
    
    // Park变换 (Ialpha, Ibeta -> Iq)
    float ct = cos(angle_el);
    float st = sin(angle_el);
    
		I_q = I_beta * ct - I_alpha * st;
    // float I_d = I_alpha * ct + I_beta * st; // 需要Id控制
    
    // 低通滤波
    I_q = LowPassFilter_Update(&current_q_filter, I_q);
    
    // PID
    float Uq = Current_Control(target_current - I_q);
    
    // 设置扭矩
    setTorque(Uq, 0, angle_el);
}




