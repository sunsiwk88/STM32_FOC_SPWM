#include "pid.h"

float Angle_Out;//角度PWM输出
float Speed_Out;//速度PWM输出

pid pid_speed;//定义速度环pid
pid pid_angle;//定义角度环pid
pid pid_current;//定义电流环pid
/**********************
PID配置函数:
**********************/
void PID_init()//初始化pid变量
{
	//位置环pid
	pid_angle.Set=0.0;
	pid_angle.Actual=0.0;
	pid_angle.err=0.0;
	pid_angle.err_last=0.0;
	pid_angle.voltage=0.0;
	pid_angle.integral=0.0;
	pid_angle.Kp=8.0;//待调节0.043
	pid_angle.Ki=0.005;//待调节0.0005
	pid_angle.Kd=0.3;//待调节0.22
	//速度环pid
	pid_speed.Set=0.0;
	pid_speed.Actual=0.0;
	pid_speed.err=0.0;
	pid_speed.err_last=0.0;
	pid_speed.voltage=0.0;
	pid_speed.integral=0.0;	
//	pid_speed.Kp=0.11;//待调节0.11
//	pid_speed.Ki=0.005;//待调节0.005
//	pid_speed.Kd=0.0008;//待调节0.0008
	
	 // 1kHz控制频率下，参数需要相应调整
    // 原来100Hz: Kp=0.11, Ki=0.005, Kd=0.0008
    // 1kHz(10倍频率): 参数应缩小
    pid_speed.Kp = 0.13;   // 减小P，避免震荡0.15
    pid_speed.Ki = 0.0007; // 减小I，防止积分饱和过快0.0005
    pid_speed.Kd = 0.0005;  // 可适当增大D，改善动态响应
		
		// 电流环 PID 初始化 (参数参考 Arduino 代码: P=1.2, I=0, D=0)
    pid_current.Set = 0.0;
    pid_current.Actual = 0.0;
    pid_current.err = 0.0;
    pid_current.err_last = 0.0;
    pid_current.voltage = 0.0;
    pid_current.integral = 0.0;
    
    pid_current.Kp = 0.2f;    // P参数
    pid_current.Ki = 0.07f;    // I参数
    pid_current.Kd = 0.0f;    // D参数
}


/**********************
角度环：pid控制

输入：角度误差
输出：角度环输出(控制力矩)
**********************/
float Angle_Control(float Angle_Err)
{
//	int PWM_Out;

//	pid_angle.err=Angle_Err;
//	
//	pid_angle.integral+=pid_angle.err;
//	
//	PWM_Out=pid_angle.Kp * pid_angle.err+ pid_angle.Ki * pid_angle.integral + pid_angle.Kd * (pid_angle.err-pid_angle.err_last);

//	pid_angle.integral=pid_angle.integral>2000?2000:(pid_angle.integral<(-2000)?(-2000):pid_angle.integral);//积分限幅
//	
//	pid_angle.err_last=pid_angle.err;
//	
//	return PWM_Out;
	
	float output;
    
    pid_angle.err = Angle_Err;
    
    // 积分项（带抗饱和）
    pid_angle.integral += pid_angle.err;
    if(pid_angle.integral > 50.0f) pid_angle.integral = 50.0f;
    if(pid_angle.integral < -50.0f) pid_angle.integral = -50.0f;
    
    // PID计算（输出速度 rad/s）
    output = pid_angle.Kp * pid_angle.err + 
             pid_angle.Ki * pid_angle.integral + 
             pid_angle.Kd * (pid_angle.err - pid_angle.err_last);
    
    pid_angle.err_last = pid_angle.err;
    
    return output;
}
/**********************
速度环：pid控制

输入：速度误差
输出：速度环输出(控制力矩)
**********************/
float Speed_Control(float Speed_Err)
{
//	int PWM_Out;

//	pid_speed.err=Speed_Err;
//	
//	pid_speed.integral+=pid_speed.err;
//	
//	PWM_Out=pid_speed.Kp * pid_speed.err + pid_speed.Ki * pid_speed.integral + pid_speed.Kd * (pid_speed.err-pid_speed.err_last);

//	pid_speed.integral=pid_speed.integral>3000?3000:(pid_speed.integral<(-3000)?(-3000):pid_speed.integral);//积分限幅
//	
//	pid_speed.err_last=pid_speed.err;
//	
//	return PWM_Out;
//	

   	float  output;
    pid_speed.err = Speed_Err;
    pid_speed.integral += pid_speed.err;
    
    // 积分限幅
    if(pid_speed.integral > 1000.0f) pid_speed.integral = 1000.0f;
    if(pid_speed.integral < -1000.0f) pid_speed.integral = -1000.0f;
    
    output = pid_speed.Kp * pid_speed.err + 
              pid_speed.Ki * pid_speed.integral + 
              pid_speed.Kd * (pid_speed.err - pid_speed.err_last);
    
    pid_speed.err_last = pid_speed.err;
    
    return output;
}


// 添加电流环控制函数
// 输入：电流误差 (目标Iq - 实际Iq)
// 输出：电压 Uq
float Current_Control(float Current_Err)
{
    float output;
    pid_current.err = Current_Err;
    
    // 积分项
    pid_current.integral += pid_current.err;
    
    // 积分限幅 (根据实际电源电压调整，例如电压的一半)
    if(pid_current.integral > 12.0f) pid_current.integral = 12.0f;
    if(pid_current.integral < -12.0f) pid_current.integral = -12.0f;
    
    output = pid_current.Kp * pid_current.err + 
             pid_current.Ki * pid_current.integral + 
             pid_current.Kd * (pid_current.err - pid_current.err_last);
    
    pid_current.err_last = pid_current.err;
    
    return output;
}


