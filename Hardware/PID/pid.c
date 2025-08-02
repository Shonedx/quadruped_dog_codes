#include "Allheaderfile.h"
#include "pid.h"
#include "motor.h"
//������һЩ�ṹ��
Motor_Speed_Loop_Pid  motor_speed_loop_pid; //�ٶȻ��ṹ��
Motor_Position_Loop_Pid motor_position_loop_pid; //�ǶȻ��ṹ��
IMU_Euler_Angle_Pid imu_euler_angle_pid; //imuŷ����pid�������ṹ��

Motor_Final_Output_Angles motor_final_output_angles={0}; //����������������Ŀ����ԽǶȣ�absolute angle)�Ľṹ������

float constrain(float value, float min, float max) { //�޷�����
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
void Pid_Speed_Loop_Init(Motor_Speed_Loop_Pid *pid) //�ٶȻ���ʼ��
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
void Pid_Position_Loop_Init(Motor_Position_Loop_Pid *pid) //λ�û���ʼ��
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
void IMU_Euler_Angle_Pid_Init(IMU_Euler_Angle_Pid *pid) //ŷ����pid����ʼ��
{
	/***Yaw***/
		pid->Yaw.Kp=0.5;//8.2
		
		pid->Yaw.Ki =0;
		
		pid->Yaw.Kd  =0;//2.81
		  
		pid->Yaw.max_out =0.52f;//16384
		
		pid->Yaw.mode =PID_POSITION;
	
	/***Pitch***/	
		pid->Pitch.Kp=0.5;//8.2
		
		pid->Pitch.Ki =0;
		
		pid->Pitch.Kd  =0;//2.81
		  
		pid->Pitch.max_out =0.52f;//16384
		
		pid->Pitch.mode =PID_POSITION;
	
	/***Roll***/
		pid->Roll.Kp=0.5;//8.2
		
		pid->Roll.Ki =0;
		
		pid->Roll.Kd  =0;//2.81
		  
		pid->Roll.max_out =0.52f;//16384
		
		pid->Roll.mode =PID_POSITION;
	
}
void PID_Init(void) //pid��ʼ��
{
    Pid_Speed_Loop_Init(&motor_speed_loop_pid); 
	Pid_Position_Loop_Init(&motor_position_loop_pid);
	IMU_Euler_Angle_Pid_Init(&imu_euler_angle_pid);
}
float IMU_pidCal(Pid_Property* pid,float fdb,float trg)
{
	pid->error[2]=pid->error[1];
	pid->error[1]=pid->error[0];
	pid->set=trg;
	pid->fdb=fdb;
	pid->error[0]=pid->fdb-pid->set;
	pid->Dbuf[2]=pid->Dbuf[1];
	pid->Dbuf[1]=pid->Dbuf[0];
	pid->Dbuf[0]=(pid->error[0]-pid->error[1]);
	
	pid->Pout=pid->Kp*pid->error[0];
	pid->Iout+=pid->Ki*pid->error[0];
	pid->Dout=pid->Kd*pid->Dbuf[0];

	pid->Iout=constrain(pid->Iout,-pid->max_iout,pid->max_iout);
	pid->out=pid->Pout+pid->Iout+pid->Dout;
	pid->out=constrain(pid->out,-pid->max_out,pid->max_out);
	return pid->out;
}

float PID_Calc( Pid_Property *pid,Motor_Property *motor_msgs) // PID���㹫ʽ
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
   

    if (pid->mode == PID_POSITION) //λ��ʽ //����ֻ������ǶȻ�
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
    else if (pid->mode == PID_INCREMENTAL)//����ʽ
    {
		pid->set =  motor_msgs->target_speed; //����ֻ������ʽ���ٶȻ�
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



void Set_Angle_Loop_Parameters(Motor_Property *motor,float target_angle,float current_angle) //���ýǶȻ���ز��������ﴫ���˽ǶȻ���Ŀ��ǶȺ͵�ǰ�Ƕ�
{
	motor->target_angle = target_angle;
	motor->current_angle= current_angle;
}

void PID_Setting(Pid_Property *pid,float kp,float kd,float ki) //����pid P I D ��Ӧ����
{
	pid->Kp=kp;
	pid->Kd=kd;
	pid->Ki=ki;
}

void ChangeTheGainOfPID_KP_KI_KD(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd) //���ýǶȻ����ٶȻ��Ķ�Ӧ����
{
	for(int i=0;i<8;i++)
	{
		PID_Setting(&motor_speed_loop_pid.ID[i],sp_kp,sp_kd,sp_ki);//speed
		PID_Setting(&motor_position_loop_pid.ID[i],pos_kp,pos_kd,pos_ki);//pos
	}
}
void changePosSpdPID(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd,uint8_t i)
{
	PID_Setting(&motor_speed_loop_pid.ID[i],sp_kp,sp_kd,sp_ki);
	PID_Setting(&motor_position_loop_pid.ID[i],pos_kp,pos_kd,pos_ki);
} 
void changePIDForSingleLeg(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd,uint8_t leg_id)
{
	switch (leg_id)
	{
	case 0 :
		changePosSpdPID(sp_kp,sp_ki,sp_kd,pos_kp,pos_ki,pos_kd,0);
		changePosSpdPID(sp_kp,sp_ki,sp_kd,pos_kp,pos_ki,pos_kd,1);
		break;
	case 1 :
		changePosSpdPID(sp_kp,sp_ki,sp_kd,pos_kp,pos_ki,pos_kd,2);
		changePosSpdPID(sp_kp,sp_ki,sp_kd,pos_kp,pos_ki,pos_kd,3);
		break;
	case 2 :
		changePosSpdPID(sp_kp,sp_ki,sp_kd,pos_kp,pos_ki,pos_kd,4);
		changePosSpdPID(sp_kp,sp_ki,sp_kd,pos_kp,pos_ki,pos_kd,5);
		break;
	case 3 :
		changePosSpdPID(sp_kp,sp_ki,sp_kd,pos_kp,pos_ki,pos_kd,6);
		changePosSpdPID(sp_kp,sp_ki,sp_kd,pos_kp,pos_ki,pos_kd,7);
		break;
	default:
		break;
	}
}
