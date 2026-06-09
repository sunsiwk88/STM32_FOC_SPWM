#ifndef  _PID_H
#define  _PID_H

typedef struct//PID结构体
{
	float Set;//目标值
	float Actual;//实际值
	float err;//误差值
	float err_last;//上一次误差值
	float Kp,Ki,Kd;//PID参数
	float voltage;//电压值
	float integral;//积分值
}pid;

extern pid pid_speed;//速度PID
extern pid pid_angle;//角度PID
extern pid pid_current;//电流PID

void PID_init(void);//PID初始化

float Angle_Control(float Angle_Err);
float Speed_Control(float Speed_Err);
float Current_Control(float Current_Err);

               
#endif
