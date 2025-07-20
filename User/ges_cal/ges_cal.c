#include "Allheaderfile.h"
#include "ges_cal.h"
#include "gaitparams.h"
#include "pid.h"
#include "RC.h"
#include "can.h"
//  LF(0)--------------------RF(3)
//  motor1       |             motor3             
//    (id:1)     |               (id:3)
//  motor2       |             motor4
//    (id:2)     |                (id:4)
//               |
//               |
//               |
//  LB(1)--------------------RB(2)
//   motor5                     motor7
//     (id:1)                     (id:3)
//   motor6                    motor8
//     (id:2)                     (id4)
//
// 2023-3-16  Motor_id_Sign



extern MotionState_t current_motion_state;
extern Motor_Final_Output_Angles motor_final_output_angles; 
extern 	int start;
extern CtrlState_t ctrl_state;
extern uint8_t setted_height;
extern JumpState_t jump_state;

Leg legs[4];

//rotateAndStretch
//声明函数

//声明结构体
Rotate_Stretch_t rotate_stretch_struct = { 
	.rotate_angle = 0.0f, // 旋转角度
	.stretch_length = 0.0f, // 伸展长度
	.rotate_time = 0.25f, // 旋转时间
	.stretch_time = 0.25f, // 伸展时间
	.rotate_count = 0.0f, //计数值
	.stretch_count = 0.0f, 
	.rotate_prev_t = 0.0f, // 上一次计数时间
	.stretch_prev_t = 0.0f, // 上一次计数时间
	.flag = 0, // 完成置位
	.rotate_freq = 8.5f, // 旋转频率
	.stretch_freq= 8.5f // 伸展频率
};
 /******************************************************************************************************/
 static float constrain(float value,float min,float max)
 {
	if(value<min) value=min;
	if(value>max) value=max;
	return value;
}
static void SinTrajectory(double t, GaitParams gait_params,Leg *leg)
{
	double x, z, gp;
	// volatile static double p = 0;
	// volatile static double prev_t = 0;
	leg->p += gait_params.freq * (t - leg->prev_t);
	leg->prev_t = t;
	gp = fmod(leg->p + gait_params.gaitoffset, 1);
	if(current_motion_state==MS_STOP)
    {
       leg-> p=0;
       leg-> prev_t = t;
    }

	if (gp <= gait_params.swingpercent) {
		x = (gp / gait_params.swingpercent) * gait_params.steplength - gait_params.steplength / 2.0+ gait_params.x_offset;
		z = -gait_params.Up_Amp * sin(PI * gp / gait_params.swingpercent) + gait_params.stanceheight;
	}
	else {
		float percentBack = (gp - gait_params.swingpercent) / (1.0 - gait_params.swingpercent);
		x = -percentBack * gait_params.steplength + gait_params.steplength / 2.0 + gait_params.x_offset;
		z = gait_params.Down_Amp * sin(PI * percentBack) + gait_params.stanceheight;
	}
	legs[gait_params.i].x = x;
	legs[gait_params.i].z = z;
} 


void Stand_Init(void)
{
	
	Set_Max_Output_SL(10000);
	Set_Max_Output_PL(10000);
	ChangeTheGainOfPID_KP_KI_KD(1,0,0.5,1,0,0);

	motor_final_output_angles.ID[0] =	-0.0f * Gaito;  //�����������⺯�������ĽǶȸ���������Ҫ����PID���������м��������
	motor_final_output_angles.ID[1] =	-180.0f * Gaito;
		
	motor_final_output_angles.ID[2] =	180.0f * Gaito;  
	motor_final_output_angles.ID[3] =	-0.0f * Gaito;
	
	motor_final_output_angles.ID[4] =	-180.0f * Gaito;
	motor_final_output_angles.ID[5] =	-0.0f * Gaito;
	
	motor_final_output_angles.ID[6] =	-180 * Gaito;  
	motor_final_output_angles.ID[7] =	-0.0f * Gaito;
	
	Motor_Auto_Run();
	
	
}
//	CS_NONE,
//	CS_INIT,
//	CS_MAIN,
//	CS_PRE_JUMP,
//	CS_EXE_JUMP,
//	CS_HEIGHT,
//	CS_QUIT,


void motion_state_ctrl(void)
{
	switch(ctrl_state)
	{
		case CS_NONE://��ʼ״̬ 
			start=0;
			break;
		case CS_INIT://��ʼ��������
			start=0;
			M3508_ALL_ZERO_SET();
			break;
		case CS_MAIN://������
			RC_MotionCtrl();
			start=1;
			break;
		case CS_PRE_JUMP:// ��Ծ׼��
			start=1;
			break;
		case CS_EXE_JUMP:// ��Ծִ��
			start=1;
			break;
		case CS_HEIGHT:
			Set_StandHeight(gait_params[0],setted_height); //NORMAL
			Set_StandHeight(gait_params[1],setted_height);//STOP
			Set_StandHeight(gait_params[2],setted_height);//TRANS_LEFT
			Set_StandHeight(gait_params[3],setted_height);//TRANS_RIGHT
			Set_StepLength(gait_params[0]);
			Set_StepLength(gait_params[1]);
			Set_UpAmpLength(gait_params[0]);
			Set_UpAmpLength(gait_params[1]);
			Set_UpAmpLength(gait_params[2]);
			Set_UpAmpLength(gait_params[3]);
			start=1;
			break;
		case CS_QUIT:
			start=0;
			break;
		default:
			break;
	}


}
 void Gait(double t)
{
	
	switch (current_motion_state)
	{
		if(ctrl_state!=CS_HEIGHT)
		{
			case MS_NORMAL:
				Set_Max_Output_SL(8000);
				Set_Max_Output_PL(8000);
				ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
				RC_StepLengthCtrl(gait_params[0]);
				for (int i=0 ; i < 4; i++)
				{
					SinTrajectory(t, gait_params[0][i],&legs[i]);
					//rotate_stretch_struct.flag=rotateAndStretch(t,  -30, 32,&legs[i], &rotate_stretch_struct,&gait_params[0][i]);
				}
				break;
			if(if_in_normal_range(setted_height, 14, 30)) //�߶Ⱥ���ʱ
			{
				case MS_TRANSLATE_LEFT:
					Set_Max_Output_SL(10000);
					Set_Max_Output_PL(10000);
					ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
					// ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
					for (int i = 0; i < 4; i++)
					{
						SinTrajectory(t, gait_params[2][i],&legs[i]);
					}
					break;
				
				case MS_TRANSLATE_RIGHT:
					Set_Max_Output_SL(10000);
					Set_Max_Output_PL(10000);
					ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
					// ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
					for (int i = 0; i < 4; i++)
					{
						SinTrajectory(t, gait_params[3][i],&legs[i]);
					}
					break;
			}
		}
		case MS_STOP:
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
			// ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
			for (int i = 0; i < 4; i++)
			{
				legs[gait_params[1][i].i].x = 0;
				legs[gait_params[1][i].i].z = gait_params[1][i].stanceheight;
			}
			// if(rotate_stretch_struct.flag==1) //如果旋转伸展完成
			// {
			// 	rotate_stretch_struct.flag=0; //归零
			// }
			updatePrevTime(t);//更新上一次计数时间
			break;
		
	
	}
	jump_state=IDLE;
	
	CartesianToTheta_Cycloid_All_Legs();
	Moveleg();
	Motor_Auto_Run();
	

}
//�˶����
/******************************************************************************************************/
// ��⺯��
void CartesianToTheta_Cycloid(Leg *leg)
{
    leg->L = sqrt(leg->x * leg->x + leg->z * leg->z);
	if(leg->L<CrouchHeight) leg->L=CrouchHeight;
	if(leg->L>HeigherHeight) leg->L=HeigherHeight;
    leg->psai1 = asin(leg->x / leg->L);
    leg->fai1 = acos((leg->L * leg->L + L1 * L1 - L2 * L2) / (2 * L1 * leg->L));
    leg->theta2 = 180.0f * (leg->fai1 - leg->psai1) / PI - 90.0f;
    leg->theta1 = 180.0f * (leg->fai1 + leg->psai1) / PI - 90.0f;
}
////���������
void CartesianToTheta_Cycloid_All_Legs(void)
{
	for(int i=0;i<4;i++)
	{
		CartesianToTheta_Cycloid(&legs[i]);
	}
}
//������ذڶ���
void Angle_Setting_Cycloid(int LegID)  // Moveleg �ﱻ����
{ 
	switch (LegID)
	{
		case 0:
			motor_final_output_angles.ID[0] =	legs[0].theta1 * Gaito;  //�����������⺯�������ĽǶȸ���������Ҫ����PID���������м��������
			motor_final_output_angles.ID[1] =	legs[0].theta2 * Gaito;
		break;
		case 1:
			motor_final_output_angles.ID[2] =	-legs[1].theta2 * Gaito;  
			motor_final_output_angles.ID[3] =	-legs[1].theta1 * Gaito;
		break;
		case 2:
			motor_final_output_angles.ID[4] =	legs[2].theta1 * Gaito;
			motor_final_output_angles.ID[5] =	legs[2].theta2 * Gaito;
		break;
		case 3:
			motor_final_output_angles.ID[6] =	-legs[3].theta2 * Gaito;  
			motor_final_output_angles.ID[7] =	-legs[3].theta1 * Gaito;
		break;
	}
}
// �ƶ��Ⱥ���
void Moveleg(void)
{
    // ����˵����
    // direction: �����־
    // Ϊÿ�������ýǶ�
    // Legid �� 0 �� 3 �ֱ����������
    Angle_Setting_Cycloid(0);  // ���õ� 0 ���ȵĽǶ�
    Angle_Setting_Cycloid(1);  // ���õ� 1 ���ȵĽǶ�
    Angle_Setting_Cycloid(2);  // ���õ� 2 ���ȵĽǶ�
    Angle_Setting_Cycloid(3);  // ���õ� 3 ���ȵĽǶ�
}

//定义函数结构体
u8 rotateAndStretch(float t,  float rotate_angle, float stretch_length,float original_length,float original_angle,Leg *leg,Rotate_Stretch_t *rs)
{
	
	if(rs->flag) //如果已置位则归零所有prev_t
	{
		rs->rotate_prev_t[0] = 0;
		rs->stretch_prev_t[0] = 0;
		rs->rotate_prev_t[1] =0;
		rs->stretch_prev_t[1] =0;
		return 1; //标志位已置位，需要正常使用该函数需将其归零
	}
	if(!rs->rotate_prev_t[0]
		&&!rs->stretch_prev_t[0]
		&&!rs->rotate_prev_t[1]
		&&!rs->stretch_prev_t[1]) //如果是第一次进入该函数
	{
		rs->rotate_prev_t[0]=t;
		rs->stretch_prev_t[0]=t;
		return 0; //立刻返回0
	}
	//判断时间是否合适
	if(rs->rotate_time<0) rs->rotate_time=0;
	if(rs->stretch_time<0) rs->stretch_time=0;
	if(rs->rotate_time>1) rs->rotate_time=1;
	if(rs->stretch_time>1) rs->stretch_time=1;
	//
	rs->rotate_count += rs->rotate_freq*(t - rs->rotate_prev_t[0]);
	rs->stretch_count += rs->stretch_freq*(t - rs->stretch_prev_t[0]);
	rs->rotate_prev_t[0] = t;
	rs->stretch_prev_t[0] = t;
	rs->rotate_prev_t[1] = rs->rotate_prev_t[0];
	rs->stretch_prev_t[1] = rs->stretch_prev_t[0];
	
	rs->rotate_count=constrain(rs->rotate_count, 0, rs->rotate_time); //旋转和伸展部分时间分开计时，默认二者是相同时间
	rs->rotate_count=constrain(rs->rotate_count, 0, rs->stretch_time);

	rs->stretch_k=rs->stretch_count/rs->stretch_time;
	rs->rotate_k=rs->rotate_count/rs->rotate_time;
	leg->x =(rs->stretch_k *stretch_length+(1-rs->stretch_k)*original_length) * sin((rs->rotate_k * rotate_angle +(1-rs->rotate_k)*original_angle)* PI / 180.0f);
	leg->z =(rs->stretch_k *stretch_length+(1-rs->stretch_k)*original_length) * cos((rs->rotate_k * rotate_angle +(1-rs->rotate_k)*original_angle) * PI / 180.0f);

	if(rs->rotate_k>=1&&rs->stretch_k>=1) //旋转和伸展都完成
	{
		rs->rotate_count=0;
		rs->stretch_count=0;
		rs->flag=1; //结构体的flag位置位完成
		return 1; //返回1表示完成 外部使用可以用别的变量接收置位 
	}
	return 0; //返回0表示未完成
}
void updatePrevTime(float t)
{
	// rotate_stretch_struct.rotate_prev_t = t;
	// rotate_stretch_struct.stretch_prev_t = t;
	for(u8 i;i<4;i++)
	{
		legs[i].prev_t = t;
	}
}
