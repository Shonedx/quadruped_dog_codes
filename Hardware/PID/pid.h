#ifndef __PID_H
#define __PID_H
#include "stdint.h"
#include "motor.h"
 enum PID_MODE //pidλ�û����ٶȻ���ѡ��
{
    PID_POSITION = 0,
    PID_INCREMENTAL     = 1
};

typedef struct
{
    uint8_t mode;           // ����ģʽ��PID_POSITION �� PID_DELTA

    float Kp;               // ��������
    float Ki;               // ��������
    float Kd;               // ΢������

    float max_out;          // ������
    float max_iout;         // ���������

    float set;              // �趨ֵ
    float fdb;              // ����ֵ

    float out;              // �����
    float Pout;             // ���������
    float Iout;             // ���������
    float Dout;             // ΢�������
    float Dbuf[3];          // ΢�����
    float error[3];         // ����
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
}Motor_Final_Output_Angles;  			//���ȵ���Ƕ�

//�������pid.c�ļ���������
extern Motor_Speed_Loop_Pid  motor_speed_loop_pid;
extern Motor_Position_Loop_Pid motor_position_loop_pid;
extern IMU_Euler_Angle_Pid imu_euler_angle_pid;

extern Motor_Final_Output_Angles motor_final_output_angles;

/***********************************/
float constrain(float value, float min, float max); //�޷�����

void Pid_Speed_Loop_Init(Motor_Speed_Loop_Pid *pid); //�ٶȻ���ʼ��

void Pid_Position_Loop_Init(Motor_Position_Loop_Pid *pid); //λ�û���ʼ��

void IMU_Euler_Angle_Pid_Init(IMU_Euler_Angle_Pid *pid); //ŷ����pid����ʼ��

void PID_Init(void); //pid��ʼ��

float PID_Calc( Pid_Property *pid,Motor_Property *motor_msgs); // �ٶȻ�PID��ʽ

void Set_Angle_Loop_Parameters(Motor_Property *motor,float target_angle,float current_angle); //���ýǶȻ���ز��������ﴫ���˽ǶȻ���Ŀ��ǶȺ͵�ǰ�Ƕ�

void PID_Setting(Pid_Property *pid,float kp,float kd,float ki); //����pid P I D ��Ӧ����

void ChangeTheGainOfPID_KP_KI_KD(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd); //���ýǶȻ����ٶȻ��Ķ�Ӧ����

void changePIDForSingleLeg(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd,uint8_t leg_id);

#endif
