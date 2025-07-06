#ifndef __PID_H
#define __PID_H
#include "stdint.h"
#include "motor.h"
 enum PID_MODE //pid位置环和速度环的选择
{
    PID_POSITION = 0,
    PID_INCREMENTAL     = 1
};

typedef struct
{
    uint8_t mode;           // 控制模式：PID_POSITION 或 PID_DELTA

    float Kp;               // 比例增益
    float Ki;               // 积分增益
    float Kd;               // 微分增益

    float max_out;          // 最大输出
    float max_iout;         // 最大积分输出

    float set;              // 设定值
    float fdb;              // 反馈值

    float out;              // 总输出
    float Pout;             // 比例项输出
    float Iout;             // 积分项输出
    float Dout;             // 微分项输出
    float Dbuf[3];          // 微分项缓存
    float error[3];         // 误差缓存
} Pid_Property;


typedef struct 
{
	Pid_Property ID[8];
}Motor_Speed_Loop_Pid;
typedef struct 
{
	Pid_Property ID[8];
}Motor_Position_Loop_Pid;
typedef struct 
{
	Pid_Property Yaw;
	Pid_Property Pitch;
	Pid_Property Roll;
}IMU_Euler_Angle_Pid;

typedef struct
{
	float ID[8];
}Motor_Final_Output_Angles;  			//狗腿电机角度

//这个都在pid.c文件中声明了
extern Motor_Speed_Loop_Pid  motor_speed_loop_pid;
extern Motor_Position_Loop_Pid motor_position_loop_pid;
extern IMU_Euler_Angle_Pid imu_euler_angle_pid;

extern Motor_Final_Output_Angles motor_final_output_angles;

/***********************************/
float constrain(float value, float min, float max); //限幅函数

void Pid_Speed_Loop_Init(Motor_Speed_Loop_Pid *pid); //速度环初始化

void Pid_Position_Loop_Init(Motor_Position_Loop_Pid *pid); //位置环初始化

void IMU_Euler_Angle_Pid_Init(IMU_Euler_Angle_Pid *pid); //欧拉角pid环初始化

void PID_Init(void); //pid初始化

float PID_Calc( Pid_Property *pid,Motor_Property *motor_msgs); // 速度环PID公式

void Set_Angle_Loop_Parameters(Motor_Property *motor,float target_angle,float current_angle); //设置角度环相关参数，这里传入了角度环的目标角度和当前角度

void PID_Setting(Pid_Property *pid,float kp,float kd,float ki); //设置pid P I D 对应参数

void ChangeTheGainOfPID_KP_KI_KD(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd); //设置角度环和速度环的对应参数

#endif
