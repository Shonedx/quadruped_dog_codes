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

	//from timer.c
	extern double now_time;
	//from jump.c

	extern uint8_t Jump_OK ;
	//from pid.c
	extern Motor_Final_Output_Angles motor_final_output_angles; 
	Leg legs[4];
//步态相关	
 /******************************************************************************************************/

void SinTrajectory(double t, GaitParams gait_params)
{
	double x, z, gp;
	static double p = 0;
	static double prev_t = 0;
	p += gait_params.freq * (t - prev_t);
	prev_t = t;
	gp = fmod(p + gait_params.gaitoffset, 1);
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
//	CS_NONE,
//	CS_INIT,
//	CS_MAIN,
//	CS_PRE_JUMP,
//	CS_EXE_JUMP,
//	CS_HEIGHT,
//	CS_QUIT,
extern 	int start;
extern CtrlState_t ctrl_state;
extern uint8_t setted_height;

void motion_state_ctrl(void)
{
	switch(ctrl_state)
	{
		case CS_NONE://初始状态 
			start=0;
			break;
		case CS_INIT://初始化机器狗
			start=0;
			M3508_ALL_ZERO_SET();
			break;
		case CS_MAIN://主控制
			RC_MotionCtrl();
			start=1;
			break;
		case CS_PRE_JUMP:// 跳跃准备
			start=1;
			break;
		case CS_EXE_JUMP:// 跳跃执行
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
extern JumpState_t jump_state;
 void Gait(double t)
{
	
	switch (current_motion_state)
	{
		if(ctrl_state!=CS_HEIGHT)
		{
			case MS_NORMAL:
				Set_Max_Output_SL(8000);
				Set_Max_Output_PL(8000);
				ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
				RC_StepLengthCtrl(gait_params[0]);
				for (int i=0 ; i < 4; i++)
				{
					SinTrajectory(t, gait_params[0][i]);
				}
				break;
			if(if_in_normal_range(setted_height, 14, 30)) //高度合适时
			{
				case MS_TRANSLATE_LEFT:
					Set_Max_Output_SL(10000);
					Set_Max_Output_PL(10000);
					ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
					for (int i = 0; i < 4; i++)
					{
						SinTrajectory(t, gait_params[2][i]);
					}
					break;
				
				case MS_TRANSLATE_RIGHT:
					Set_Max_Output_SL(10000);
					Set_Max_Output_PL(10000);
					ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
					for (int i = 0; i < 4; i++)
					{
						SinTrajectory(t, gait_params[3][i]);
					}
					break;
			}
		}
		case MS_STOP:
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
			now_time=0;
			for (int i = 0; i < 4; i++)
			{
				legs[gait_params[1][i].i].x = 0;
				legs[gait_params[1][i].i].z = gait_params[1][i].stanceheight;
			}
			
			break;
		
	
	}
	jump_state=IDLE;
	
	// 转换到逆运动学的角度
	CartesianToTheta_Cycloid_All_Legs();
	// 控制腿部运动
	Moveleg();
	Motor_Auto_Run();
	

}
//运动相关
/******************************************************************************************************/
// 逆解函数
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
