#include "Allheaderfile.h"
#include<stdlib.h>
#include "stdio.h"
#include "math.h"

//声明了一些结构体
Motor_Speed_Loop_Pid  motor_speed_loop_pid; //速度环结构体
Motor_Position_Loop_Pid motor_position_loop_pid; //角度环结构体
IMU_Euler_Angle_Pid imu_euler_angle_pid; //imu欧拉角pid控制器结构体

Motor_Final_Output_Angles motor_final_output_angles={0}; //储存最终输出电机的目标绝对角度（absolute angle)的结构体数组

float constrain(float value, float min, float max) { //限幅函数
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
void Pid_Speed_Loop_Init(Motor_Speed_Loop_Pid *pid) //速度环初始化
{
	for(int i=0;i<8;i++)
	{
		pid->ID[i].Kp=8.2f;//8.2
		
		pid->ID[i].Ki =0.2f;
		
		pid->ID[i].Kd  =2.81f;//2.81
		  
		pid->ID[i].max_out =10000;//16384
		
		pid->ID[i].mode =PID_INCREMENTAL;
		
	}
}
void Pid_Position_Loop_Init(Motor_Position_Loop_Pid *pid) //位置环初始化
{
	for(int i=0;i<8;i++)
	{
		pid->ID[i].Kp=7.1f;//8.2
		
		pid->ID[i].Ki =0.12f;
		
		pid->ID[i].Kd  =1.82f;//2.81
		  
		pid->ID[i].max_out =6000;//16384
		
		pid->ID[i].mode =PID_POSITION;
		
	}
}
void IMU_Euler_Angle_Pid_Init(IMU_Euler_Angle_Pid *pid) //欧拉角pid环初始化
{
	/***Yaw***/
		pid->Yaw.Kp=0;//8.2
		
		pid->Yaw.Ki =0;
		
		pid->Yaw.Kd  =0;//2.81
		  
		pid->Yaw.max_out =0.52f;//16384
		
		pid->Yaw.mode =PID_POSITION;
	
	/***Pitch***/	
		pid->Pitch.Kp=0;//8.2
		
		pid->Pitch.Ki =0;
		
		pid->Pitch.Kd  =0;//2.81
		  
		pid->Pitch.max_out =0.52f;//16384
		
		pid->Pitch.mode =PID_POSITION;
	
	/***Roll***/
		pid->Roll.Kp=0;//8.2
		
		pid->Roll.Ki =0;
		
		pid->Roll.Kd  =0;//2.81
		  
		pid->Roll.max_out =0.52f;//16384
		
		pid->Roll.mode =PID_POSITION;
	
}
void PID_Init(void) //pid初始化
{
    Pid_Speed_Loop_Init(&motor_speed_loop_pid); 
	Pid_Position_Loop_Init(&motor_position_loop_pid);
	IMU_Euler_Angle_Pid_Init(&imu_euler_angle_pid);
}


float PID_Calc( Pid_Property *pid,Motor_Property *motor_msgs) // PID计算公式
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
   

    if (pid->mode == PID_POSITION) //位置式 //这里只用来算角度环
    {
		pid->set =  motor_msgs->target_angle;
		pid->fdb =  motor_msgs->current_angle;
		pid->error[0] = pid->set - pid->fdb;
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->Iout = constrain(pid->Iout, -pid->max_iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        pid->out = constrain(pid->out, -pid->max_out, pid->max_out);
    }
    else if (pid->mode == PID_INCREMENTAL)//增量式
    {
		pid->set =  motor_msgs->target_speed; //这里只用增量式算速度环
		pid->fdb =  motor_msgs->current_speed;
		pid->error[0] = pid->set - pid->fdb;
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        pid->out = constrain(pid->out, -pid->max_out, pid->max_out);
    }
    return pid->out;
}



void Set_Angle_Loop_Parameters(Motor_Property *motor,float target_angle,float current_angle) //设置角度环相关参数，这里传入了角度环的目标角度和当前角度
{
	motor->target_angle = target_angle;
	motor->current_angle= current_angle;
}

void PID_Setting(Pid_Property *pid,float kp,float kd,float ki) //设置pid P I D 对应参数
{
	pid->Kp=kp;
	pid->Kd=kd;
	pid->Ki=ki;
}

void ChangeTheGainOfPID_KP_KI_KD(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd) //设置角度环和速度环的对应参数
{
	for(int i=0;i<8;i++)
	{
		/**速度环**/
		PID_Setting(&motor_speed_loop_pid.ID[i],sp_kp,sp_kd,sp_ki);
		/**位置环**/
		PID_Setting(&motor_position_loop_pid.ID[i],pos_kp,pos_kd,pos_ki);
	}
}

