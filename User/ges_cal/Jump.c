#include "Jump.h"

int Jump_OK=1;
int Jump_Start=1;
const double prep_time=200; //准备时间  3000ms //0.3
const double execute_time_1=100; //跳跃执行时间 40ms // 0.04 
const double front_legs_execute_time=80; //前腿跳跃执行时间 0.03 30ms

const double recovery_time=200; //恢复时间 1500ms  //0.5 
const double stretch_length=18.5f; //伸展长度
const double shrink_length=6.0f; //收缩长度
const double fall_length=5.5f ;//落地缓冲长度
static uint16_t jump_time=0;
//prepare
//front
const double pre_jump1_front_legs_rotate_angle=-12.0f;
const double pre_jump1_behind_legs_rotate_angle=-15.0f;
//behind
const double pre_jump2_front_legs_rotate_angle=-23.0f;
const double pre_jump2_behind_legs_rotate_angle=-23.0f;
//execute
//front
const double exe_jump1_front_legs_rotate_angle=-20.0f;
const double exe_jump1_behind_legs_rotate_angle=-20.0f;
//behind
const double exe_jump2_front_legs_rotate_angle=-28.0f;
const double exe_jump2_behind_legs_rotate_angle=-28.0f;
//recovery
//front
const double rcv_jump1_front_legs_rotate_angle=3.0f;
const double rcv_jump1_behind_legs_rotate_angle=3.0f;
//behind
const double rcv_jump2_front_legs_rotate_angle=1.0f;
const double rcv_jump2_behind_legs_rotate_angle=2.0f;
void Reset_jump_time(void) 
{
    jump_time=0; //重置跳跃时间
}
static uint16_t Get_Jump_Time(void)
{
	return jump_time++;
}
void Execute_Jump(void) //定时器里面5ms一中断
{
    uint16_t t =Get_Jump_Time();;
   
	if(ctrl_state==Jump_Ctrl_1)
	{
			
			if (t < prep_time) {
			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			double K =t/prep_time;
			if(K>1)K=1;
//            double x = (6.0f*K+(1.0f-K)*StandHeight)*sin(K*(-15.0f)*PI/180.0f);
//            double z = (6.0f*K+(1.0f-K)*StandHeight)*cos(K*15.0f*PI/180.0f);
			double x__for_front_legs = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*pre_jump1_front_legs_rotate_angle*PI/180.0f);
			double z__for_front_legs = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*pre_jump1_front_legs_rotate_angle*PI/180.0f);
			double x_for_back_legs = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*pre_jump1_behind_legs_rotate_angle*PI/180.0f);
			double z_for_back_legs = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*pre_jump1_behind_legs_rotate_angle*PI/180.0f);
//			
				legs[2].x = x_for_back_legs;
				legs[2].z = z_for_back_legs;
				legs[3].x = x_for_back_legs;
				legs[3].z = z_for_back_legs;
				legs[0].x = x__for_front_legs;
				legs[0].z = z__for_front_legs;
				legs[1].x = x__for_front_legs;
				legs[1].z = z__for_front_legs;
			// 转换到逆运动学的角度
			CartesianToTheta_Cycloid_All_Legs();
			// 控制腿部运动
			Moveleg();
			Motor_Auto_Run();
           //printf("waiting\n");
			}
			else if ( t < prep_time+execute_time_1) {
				ChangeTheGainOfPID_KP_KI_KD(8,0.3,1.1,22,0.3,5.5);
//				ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
				Set_Max_Output_SL(12000);
				Set_Max_Output_PL(12000);
//				double x = 19.0f*sin(-20.0f*PI/180.0f);
//				double z = 19.0f*cos(20.0f*PI/180.0f);
				double x__for_front_legs;
				double z__for_front_legs;
				if(t<prep_time+front_legs_execute_time)
				{
					 x__for_front_legs = stretch_length*sin(exe_jump1_front_legs_rotate_angle*PI/180.0f);
					 z__for_front_legs = stretch_length*cos(exe_jump1_front_legs_rotate_angle*PI/180.0f);
				}
				else
				{
					 x__for_front_legs = shrink_length*sin(rcv_jump1_front_legs_rotate_angle*PI/180.0f);
					 z__for_front_legs = shrink_length*cos(rcv_jump1_front_legs_rotate_angle*PI/180.0f);
				}
				
				double x_for_back_legs = stretch_length*sin(exe_jump1_behind_legs_rotate_angle*PI/180.0f);
				double z_for_back_legs = stretch_length*cos(exe_jump1_behind_legs_rotate_angle*PI/180.0f);
				legs[2].x = x_for_back_legs;
				legs[2].z = z_for_back_legs;
				legs[3].x = x_for_back_legs;
				legs[3].z = z_for_back_legs;
				legs[0].x = x__for_front_legs;
				legs[0].z = z__for_front_legs;
				legs[1].x = x__for_front_legs;
				legs[1].z = z__for_front_legs;
				// 转换到逆运动学的角度
				CartesianToTheta_Cycloid_All_Legs();
				// 控制腿部运动
				Moveleg();
				Motor_Auto_Run();
			}//&& t < 1.5

		else 
		{
			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
//			double K2=(t-(prep_time+execute_time_1+execute_time_2+fly_time))/(recovery_time);
		
//			double x = (StandHeight*K2+(1.0f-K2)*6.0f)*sin(0.0f*PI/180.0f);
//            double z = (StandHeight*K2+(1.0f-K2)*6.0f)*cos(0.0f*PI/180.0f);
			double x__for_front_legs = fall_length*sin(rcv_jump1_front_legs_rotate_angle*PI/180.0f);
            double z__for_front_legs = fall_length*cos(rcv_jump1_front_legs_rotate_angle*PI/180.0f);
			double x_for_back_legs = fall_length*sin(rcv_jump1_behind_legs_rotate_angle*PI/180.0f);
            double z_for_back_legs = fall_length*cos(rcv_jump1_behind_legs_rotate_angle*PI/180.0f);
//			double x = 6*sin(3.0f*PI/180.0f);
//            double z =6*cos(3.0f*PI/180.0f);
			legs[2].x = x_for_back_legs;
			legs[2].z = z_for_back_legs;
			legs[3].x = x_for_back_legs;
			legs[3].z = z_for_back_legs;
			legs[0].x = x__for_front_legs;
			legs[0].z = z__for_front_legs;
			legs[1].x = x__for_front_legs;
			legs[1].z = z__for_front_legs;
			CartesianToTheta_Cycloid_All_Legs();
			// 控制腿部运动
			Moveleg();
			Motor_Auto_Run();
			if(t>=prep_time+execute_time_1+recovery_time)
			{
				
				Reset_jump_time();
				Jump_OK = 0;	//完成跳跃置零
			}
			
			
		}
	
	}
	else if (ctrl_state == Jump_Ctrl_2)
	{
		ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			if (t < prep_time) {
			double K =t/prep_time;
			if(K>1)K=1;
            double x__for_front_legs = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*pre_jump2_front_legs_rotate_angle*PI/180.0f);
            double z__for_front_legs = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*pre_jump2_front_legs_rotate_angle*PI/180.0f);
			double x_for_back_legs = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*pre_jump2_behind_legs_rotate_angle*PI/180.0f);
            double z_for_back_legs = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*pre_jump2_behind_legs_rotate_angle*PI/180.0f);
			legs[2].x = x_for_back_legs;
			legs[2].z = z_for_back_legs;
			legs[3].x = x_for_back_legs;
			legs[3].z = z_for_back_legs;
			legs[0].x = x__for_front_legs;
			legs[0].z = z__for_front_legs;
			legs[1].x = x__for_front_legs;
			legs[1].z = z__for_front_legs;
			// 转换到逆运动学的角度
			CartesianToTheta_Cycloid_All_Legs();
			// 控制腿部运动
			Moveleg();
			Motor_Auto_Run();
           //printf("waiting\n");
        }
        else if ( t < prep_time+execute_time_1) {
			ChangeTheGainOfPID_KP_KI_KD(8,0.3,1.1,22,0.3,2.5);
//			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
			Set_Max_Output_SL(12000);
			Set_Max_Output_PL(12000);
			
			double x__for_front_legs;
			double z__for_front_legs;
			if(t<prep_time+front_legs_execute_time)
			{
				x__for_front_legs = stretch_length*sin(exe_jump2_front_legs_rotate_angle*PI/180.0f);
				z__for_front_legs = stretch_length*cos(exe_jump2_front_legs_rotate_angle*PI/180.0f);
			}
			else
			{
				x__for_front_legs = fall_length*sin(rcv_jump2_front_legs_rotate_angle*PI/180.0f);
				z__for_front_legs = fall_length*cos(rcv_jump2_front_legs_rotate_angle*PI/180.0f);
			}
						
			double x_for_back_legs = stretch_length*sin(exe_jump2_behind_legs_rotate_angle*PI/180.0f);
			double z_for_back_legs = stretch_length*cos(exe_jump2_behind_legs_rotate_angle*PI/180.0f);
			legs[2].x = x_for_back_legs;
			legs[2].z = z_for_back_legs;
			legs[3].x = x_for_back_legs;
			legs[3].z = z_for_back_legs;
			legs[0].x = x__for_front_legs;
			legs[0].z = z__for_front_legs;
			legs[1].x = x__for_front_legs;
			legs[1].z = z__for_front_legs;
			CartesianToTheta_Cycloid_All_Legs();
			// 控制腿部运动
			Moveleg();
			Motor_Auto_Run();
			
        }

		else 
		{
			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			double x__for_front_legs = fall_length*sin(rcv_jump2_front_legs_rotate_angle*PI/180.0f);
            double z__for_front_legs =fall_length*cos(rcv_jump2_front_legs_rotate_angle*PI/180.0f);
			double x_for_back_legs = fall_length*sin(rcv_jump2_behind_legs_rotate_angle*PI/180.0f);
            double z_for_back_legs = fall_length*cos(rcv_jump2_behind_legs_rotate_angle*PI/180.0f);
		
			legs[2].x = x_for_back_legs;
			legs[2].z = z_for_back_legs;
			legs[3].x = x_for_back_legs;
			legs[3].z = z_for_back_legs;
			legs[0].x = x__for_front_legs;
			legs[0].z = z__for_front_legs;
			legs[1].x = x__for_front_legs;
			legs[1].z = z__for_front_legs;
			CartesianToTheta_Cycloid_All_Legs();
			// 控制腿部运动
			Moveleg();
			Motor_Auto_Run();
			if(t>=prep_time+execute_time_1+recovery_time)
			{
				
				Reset_jump_time();
				Jump_OK = 0;	//完成跳跃置零
			}
			
			
		}
	}
}

