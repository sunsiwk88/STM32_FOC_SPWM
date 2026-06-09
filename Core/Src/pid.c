#include "pid.h"

pid pid_speed;//速度PID
pid pid_angle;//角度PID
pid pid_current;//电流PID


/**********************
PID功能:
**********************/
void PID_init()//初始化PID
{
		//位置PID
		pid_angle.Set=0.0;
		pid_angle.Actual=0.0;
		pid_angle.err=0.0;
		pid_angle.err_last=0.0;
		pid_angle.voltage=0.0;
		pid_angle.integral=0.0;
	
		pid_angle.Kp=8.0;
		pid_angle.Ki=0.005;
		pid_angle.Kd=0.3;
	
		//速度PID
		pid_speed.Set=0.0;
		pid_speed.Actual=0.0;
		pid_speed.err=0.0;
		pid_speed.err_last=0.0;
		pid_speed.voltage=0.0;
		pid_speed.integral=0.0;	
	
    pid_speed.Kp = 0.13;   
    pid_speed.Ki = 0.0007; 
    pid_speed.Kd = 0.0005;  
		
		// 电流PID
    pid_current.Set = 0.0;
    pid_current.Actual = 0.0;
    pid_current.err = 0.0;
    pid_current.err_last = 0.0;
    pid_current.voltage = 0.0;
    pid_current.integral = 0.0;
		
    pid_current.Kp = 0.33f;    
    pid_current.Ki = 0.08f;    
    pid_current.Kd = 0.0f;    
}


/**********************
角度环：pid控制

输入：角度误差
输出：角度环输出(控制力矩)
**********************/
float Angle_Control(float Angle_Err)
{
	float output;
    
    pid_angle.err = Angle_Err;

    pid_angle.integral += pid_angle.err;
    if(pid_angle.integral > 50.0f) pid_angle.integral = 50.0f;
    if(pid_angle.integral < -50.0f) pid_angle.integral = -50.0f;
    
    output = pid_angle.Kp * pid_angle.err + 
             pid_angle.Ki * pid_angle.integral + 
             pid_angle.Kd * (pid_angle.err - pid_angle.err_last);
    
    pid_angle.err_last = pid_angle.err;
    
    return output;
}
/**********************
速度PID控制

输入：速度误差
输出：速度环输出(控制力矩)
**********************/
float Speed_Control(float Speed_Err)
{

   	float  output;
    pid_speed.err = Speed_Err;
    pid_speed.integral += pid_speed.err;
    
    if(pid_speed.integral > 1000.0f) pid_speed.integral = 1000.0f;
    if(pid_speed.integral < -1000.0f) pid_speed.integral = -1000.0f;
    
    output = pid_speed.Kp * pid_speed.err + 
              pid_speed.Ki * pid_speed.integral + 
              pid_speed.Kd * (pid_speed.err - pid_speed.err_last);
    
    pid_speed.err_last = pid_speed.err;
    
    return output;
}

/**********************
电流环：pid控制

输入：电流误差(目标Iq - 实际Iq)
输出：电流环输出(控制力矩)
**********************/
float Current_Control(float Current_Err)
{
    float output;
    pid_current.err = Current_Err;
    
    // 积分
    pid_current.integral += pid_current.err;
    
    // 积分限幅
    if(pid_current.integral > 12.0f) pid_current.integral = 12.0f;
    if(pid_current.integral < -12.0f) pid_current.integral = -12.0f;
    
    output = pid_current.Kp * pid_current.err + 
             pid_current.Ki * pid_current.integral + 
             pid_current.Kd * (pid_current.err - pid_current.err_last);
    
    pid_current.err_last = pid_current.err;
    
    return output;
}


