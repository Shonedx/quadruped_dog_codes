#include "pid.h"
#include "motor.h"
#include "can.h"
//声明了一些结构体
MotorSpeedPid_t motor_speed_pid; //速度环pid
MotorPosPid_t motor_pos_pid; //位置环pid
ImuEulerAnglePid_t imu_euler_angle_pid; //欧拉角pid


static float constrain(float value, float min, float max) { //限幅函数
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void pidSpeedInit(MotorSpeedPid_t *pid) //速度环初始化
{
	for(u8 i=0;i<4;i++)
	{
        //can1
        pid->can1_motors_pid[i].Kp=8.2f;//8.2
        pid->can1_motors_pid[i].Ki =0.2f;
        pid->can1_motors_pid[i].Kd  =2.81f;//2.81
        pid->can1_motors_pid[i].max_out =10000;//16384
        pid->can1_motors_pid[i].mode =PID_INCREMENTAL; //速度环用增量式
        //can2
        pid->can2_motors_pid[i].Kp=8.2f;//8.2
        pid->can2_motors_pid[i].Ki =0.2f;
        pid->can2_motors_pid[i].Kd  =2.81f;//2.81
        pid->can2_motors_pid[i].max_out =10000;//16384
        pid->can2_motors_pid[i].mode =PID_INCREMENTAL;
	}
}
void pidPosInit(MotorPosPid_t *pid) //位置环初始化
{
	for(u8 i=0;i<4;i++)
	{
        //can1
        pid->can1_motors_pid[i].Kp=7.1f;//8.2
        pid->can1_motors_pid[i].Ki =0.12f;
        pid->can1_motors_pid[i].Kd  =1.82f;//2.81
        pid->can1_motors_pid[i].max_out =6000;//16384
        pid->can1_motors_pid[i].mode =PID_POSITION; //位置环用位置式
        //can2
        pid->can2_motors_pid[i].Kp=7.1f;//8.2
        pid->can2_motors_pid[i].Ki =0.12f;
        pid->can2_motors_pid[i].Kd  =1.82f;//2.81
        pid->can2_motors_pid[i].max_out =6000;//16384
        pid->can2_motors_pid[i].mode =PID_POSITION;
	}
}

void imuPidInit(ImuEulerAnglePid_t *pid) //欧拉角pid环初始化
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
void pidInit(void) //初始化所有pid
{
    pidSpeedInit(&motor_speed_pid); 
	pidPosInit(&motor_pos_pid);
	imuPidInit(&imu_euler_angle_pid);
}
float  pidCalculate(PidPrameter_t *pid,float tar,float fdb) // PID计算公式
{
    if (pid == 0)
    {
        return 0;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
   
    pid->tar =  tar; //目标值
    pid->fdb =  fdb; //反馈值
    pid->error[0] = pid->tar - pid->fdb;

    if (pid->mode == PID_POSITION) //位置式
    {
        if(pid->error[0] < pid->dead_zone && pid->error[0] > -pid->dead_zone) //进入死区
        {
            pid->error[0] = 0;
            pid->out = 0; //死区内输出为0
        }
        else //正常情况
        {
            pid->Dbuf[2] = pid->Dbuf[1];
            pid->Dbuf[1] = pid->Dbuf[0];
            pid->Dbuf[0] = (pid->error[0] - pid->error[1]); //当前误差减去以前的误差
        
            pid->Pout = pid->Kp * pid->error[0];
            pid->Iout += pid->Ki * pid->error[0];
            pid->Dout = pid->Kd * pid->Dbuf[0];

            pid->Iout = constrain(pid->Iout, -pid->max_iout, pid->max_iout);
            
            pid->out = pid->Pout + pid->Iout + pid->Dout;

            pid->out = constrain(pid->out, -pid->max_out, pid->max_out); //限制输出
        }
       
    }
    else if (pid->mode == PID_INCREMENTAL)//增量式 
    {
        if(pid->error[0] < pid->dead_zone && pid->error[0] > -pid->dead_zone) //进入死区
        {
            pid->error[0] = 0;
            pid->out = 0; //死区内输出为0
        }
        else
        {
            pid->Dbuf[2] = pid->Dbuf[1];
            pid->Dbuf[1] = pid->Dbuf[0];
            pid->Dbuf[0] = (pid->error[0] + pid->error[2] - 2.0f * pid->error[1]);

            pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
            pid->Iout = pid->Ki * pid->error[0];
            pid->Dout = pid->Kd * pid->Dbuf[0];

            pid->Iout = constrain(pid->Iout, -pid->max_iout, pid->max_iout);

            pid->out += pid->Pout + pid->Iout + pid->Dout;

            pid->out = constrain(pid->out, -pid->max_out, pid->max_out);
        }
       
    }
    return pid->out;
}
float speedPidCalculate(PidPrameter_t *pid,MotorData_t *motor_data) // 计算速度环输出
{
    return  pidCalculate(pid,motor_data->target_speed,motor_data->speed); //速度环输出
}
float posPidCalculate(PidPrameter_t *pid,MotorData_t *motor_data) // 计算角度环输出
{
    return pidCalculate(pid,motor_data->target_angle,motor_data->absolute_angle); //角度环输出
}

//设置pid系数
void setPidFactor(PidPrameter_t *pid,float kp,float kd,float ki) 
{
	pid->Kp=kp;
	pid->Kd=kd;
	pid->Ki=ki;
}
void setPosPidFactor(float kp,float kd,float ki) //设置角度环pid系数
{
    for(u8 i=0;i<4;i++)
    {
        setPidFactor(&motor_pos_pid.can1_motors_pid[i],kp,kd,ki); //can1
        setPidFactor(&motor_pos_pid.can2_motors_pid[i],kp,kd,ki); //can2
    }
}
void setSpeedPidFactor(float kp,float kd,float ki) //设置速度环的pid系数
{
    for(u8 i=0;i<4;i++)
    {
        setPidFactor(&motor_speed_pid.can1_motors_pid[i],kp,kd,ki); //can1
        setPidFactor(&motor_speed_pid.can2_motors_pid[i],kp,kd,ki); //can2
    }
} 
void setSpeedPosPid(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd) //设置角度环和速度环的对应参数
{
	setSpeedPidFactor(sp_kp,sp_kd,sp_ki); //设置速度环的pid系数
    setPosPidFactor(pos_kp,pos_kd,pos_ki); //设置角度环的pid系数
}

