
#include "math.h"
#include "ges_cal.h"
       
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
	


	/******************/
	int Jump_State=1;
	double jump_time=2; //跳跃周期
	double jump_freq=0.0020; //跳跃速度
	double jump_angle=31.0f; //beta角是两电机连线重点到足端的向量与竖直方向的夹角
	
	/******************/

	extern State currentstate;
	
	Leg legs[4];
//步态相关	
 /******************************************************************************************************/

void SinTrajectory(double t, GaitParams gaitparams)
{
	double x, z, gp;
	static double p = 0;
	static double prev_t = 0;
	p += gaitparams.freq * (t - prev_t);
	prev_t = t;
	gp = fmod(p + gaitparams.gaitoffset, 1);
	if (gp <= gaitparams.swingpercent) {
		x = (gp / gaitparams.swingpercent) * gaitparams.steplength - gaitparams.steplength / 2.0+ gaitparams.x_offset;
		z = -gaitparams.Up_Amp * sin(PI * gp / gaitparams.swingpercent) + gaitparams.stanceheight;
	}
	else {
		float percentBack = (gp - gaitparams.swingpercent) / (1.0 - gaitparams.swingpercent);
		x = -percentBack * gaitparams.steplength + gaitparams.steplength / 2.0 + gaitparams.x_offset;
		z = gaitparams.Down_Amp * sin(PI * percentBack) + gaitparams.stanceheight;
	}
	legs[gaitparams.i].x = x;
	legs[gaitparams.i].z = z;
} 


void Stand_Init(void)
{
	
	Set_Max_Output_SL(10000);
	Set_Max_Output_PL(10000);
	ChangeTheGainOfPID_KP_KI_KD(1,0,0.5,1,0,0);

	motor_final_output_angles.ID[0] =	-0.0f * Gaito;  //这个函数把逆解函数逆解出的角度给到最最终要送至PID控制器进行计算的数组
	motor_final_output_angles.ID[1] =	-180.0f * Gaito;
		
	motor_final_output_angles.ID[2] =	180.0f * Gaito;  
	motor_final_output_angles.ID[3] =	-0.0f * Gaito;
	
	motor_final_output_angles.ID[4] =	-180.0f * Gaito;
	motor_final_output_angles.ID[5] =	-0.0f * Gaito;
	
	motor_final_output_angles.ID[6] =	-180 * Gaito;  
	motor_final_output_angles.ID[7] =	-0.0f * Gaito;
	
	Motor_Auto_Run();
	
	
}
 void Gait(double t)
{
	
	switch (currentstate)
	{
		
		case Normal:
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
			Set_StepLength(gaitparams[0]);
			for (int i=0 ; i < 4; i++)
			{
				SinTrajectory(t, gaitparams[0+crouch_flag+higher_flag][i]);
			}

			break;
		
//		case Idle:  // 原地踏步
//			Set_Max_Output_SL(8000);
//			Set_Max_Output_PL(8000);
//			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
//			for (int i=0 ; i < 4; i++)
//			{
//				SinTrajectory(t, gaitparams[0][i]);
//			}
//			break;
//		case Forward:
//			Set_Max_Output_SL(8000);
//			Set_Max_Output_PL(8000);
//			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
//			for (int i = 0; i < 4; i++)
//			{
//				SinTrajectory(t, gaitparams[1+crouch_flag+higher_flag][i]);
//			}
//			break;
//		case Back:
//			Set_Max_Output_SL(8000);
//			Set_Max_Output_PL(8000);
//			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
//			for (int i = 0; i < 4; i++)
//			{
//				SinTrajectory(t, gaitparams[2+crouch_flag+higher_flag][i]);
//			}
//			break;
//		case Turn_Left:
//			Set_Max_Output_SL(8000);
//			Set_Max_Output_PL(8000);
//			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
//			for (int i = 0; i < 4; i++)
//			{
//				SinTrajectory(t, gaitparams[3+crouch_flag+higher_flag][i]);
//			}

//			break;
//		case Turn_Right:
//			Set_Max_Output_SL(8000);
//			Set_Max_Output_PL(8000);
//			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
//			for (int i = 0; i < 4; i++)
//			{
//				SinTrajectory(t, gaitparams[4+crouch_flag+higher_flag][i]);
//			}

//			break;
	
		case Stop:
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
			now_time=0;
			for (int i = 0; i < 4; i++)
			{
				SinTrajectory(t, gaitparams[1+crouch_flag+higher_flag][i]);
			}
			Jump_Start=1;
			Jump_OK = 1;
			break;
		if(crouch_flag==0&&higher_flag==0)
		{
			case Translate_Left:
				Set_Max_Output_SL(10000);
				Set_Max_Output_PL(10000);
				ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
				for (int i = 0; i < 4; i++)
				{
					SinTrajectory(t, gaitparams[2][i]);
				}
				break;
			
			case Translate_Right:
				Set_Max_Output_SL(10000);
				Set_Max_Output_PL(10000);
				ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
				for (int i = 0; i < 4; i++)
				{
					SinTrajectory(t, gaitparams[3][i]);
				}
				break;
			case Jump:
				if(ctrl_state==Jump_Ctrl_1||ctrl_state==Jump_Ctrl_2)
				{
					
					if(Jump_OK==1)
					{
						if(Jump_Start==1)
						{
							Reset_jump_time(); //重置跳跃时间
							Jump_Start=0;
						}
						Execute_Jump();
					}
					else
					{
						Set_Max_Output_SL(8000);
						Set_Max_Output_PL(8000);
						// 转换到逆运动学的角度
						CartesianToTheta_Cycloid_All_Legs();
						// 控制腿部运动
						Moveleg();
						Motor_Auto_Run();
						
					}
				}	
				
				break;
		}
	
	}
	if(currentstate!=Jump)
	{	
		// 转换到逆运动学的角度
		CartesianToTheta_Cycloid_All_Legs();
		// 控制腿部运动
		Moveleg();
		Motor_Auto_Run();
	}

}
//运动相关
/******************************************************************************************************/
// 逆解函数
void CartesianToTheta_Cycloid(Leg *leg)
{
    leg->L = sqrt(leg->x * leg->x + leg->z * leg->z);
	if(leg->L<5.0f) leg->L=5.0f;
	if(leg->L>19.5f) leg->L=19.5f;
    leg->psai1 = asin(leg->x / leg->L);
    leg->fai1 = acos((leg->L * leg->L + L1 * L1 - L2 * L2) / (2 * L1 * leg->L));
    leg->theta2 = 180.0f * (leg->fai1 - leg->psai1) / PI - 90.0f;
    leg->theta1 = 180.0f * (leg->fai1 + leg->psai1) / PI - 90.0f;
}
////逆解所有腿
void CartesianToTheta_Cycloid_All_Legs(void)
{
	for(int i=0;i<4;i++)
	{
		CartesianToTheta_Cycloid(&legs[i]);
	}
}
//设置相关摆动角
void Angle_Setting_Cycloid(int LegID)  // Moveleg 里被调用
{ 
	switch (LegID)
	{
		case 0:
			motor_final_output_angles.ID[0] =	legs[0].theta1 * Gaito;  //这个函数把逆解函数逆解出的角度给到最最终要送至PID控制器进行计算的数组
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
// 移动腿函数
void Moveleg(void)
{
    // 参数说明：
    // direction: 方向标志
    // 为每条腿设置角度
    // Legid 从 0 到 3 分别代表四条腿
    Angle_Setting_Cycloid(0);  // 设置第 0 条腿的角度
    Angle_Setting_Cycloid(1);  // 设置第 1 条腿的角度
    Angle_Setting_Cycloid(2);  // 设置第 2 条腿的角度
    Angle_Setting_Cycloid(3);  // 设置第 3 条腿的角度
}
