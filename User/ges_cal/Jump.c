#include "Jump.h"
double start_time;
int Jump_OK=1;
int Jump_Start=1;
const double prep_time=0.3;
const double execute_time_1=0.04;
const double front_legs_execute_time=0.03;
const double fly_time=0.0;
const double recovery_time=0.5;

void Start_Jump(void)
{
    start_time = HAL_GetTick();
}
void Execute_Jump(void)
{
    double t =1.0*(HAL_GetTick()/1000.0f  - start_time/1000.0f);
   
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
			double x__for_front_legs = (6.0f*K+(1.0f-K)*StandHeight)*sin(K*(-6.0f)*PI/180.0f);
			double z__for_front_legs = (6.0f*K+(1.0f-K)*StandHeight)*cos(K*(-6.0f)*PI/180.0f);
			double x_for_back_legs = (6.0f*K+(1.0f-K)*StandHeight)*sin(K*(-12.0f)*PI/180.0f);
			double z_for_back_legs = (6.0f*K+(1.0f-K)*StandHeight)*cos(K*(-12.0f)*PI/180.0f);
//			
				legs[0].x = x_for_back_legs;
				legs[0].z = z_for_back_legs;
				legs[1].x = x_for_back_legs;
				legs[1].z = z_for_back_legs;
				legs[2].x = x__for_front_legs;
				legs[2].z = z__for_front_legs;
				legs[3].x = x__for_front_legs;
				legs[3].z = z__for_front_legs;
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
					 x__for_front_legs = 18.5f*sin(-18.0f*PI/180.0f);
					 z__for_front_legs = 18.5f*cos(18.0f*PI/180.0f);
				}
				else
				{
					 x__for_front_legs = 5*sin(3.0f*PI/180.0f);
					 z__for_front_legs = 5*cos(3.0f*PI/180.0f);
				}
				
				double x_for_back_legs = 18.5f*sin(-18.0f*PI/180.0f);
				double z_for_back_legs = 18.5f*cos(18.0f*PI/180.0f);
				legs[0].x = x_for_back_legs;
				legs[0].z = z_for_back_legs;
				legs[1].x = x_for_back_legs;
				legs[1].z = z_for_back_legs;
				legs[2].x = x__for_front_legs;
				legs[2].z = z__for_front_legs;
				legs[3].x = x__for_front_legs;
				legs[3].z = z__for_front_legs;
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
			  double x__for_front_legs = 5*sin(3.0f*PI/180.0f);
            double z__for_front_legs = 5*cos(3.0f*PI/180.0f);
			double x_for_back_legs = 6*sin(3.0f*PI/180.0f);
            double z_for_back_legs = 6*cos(3.0f*PI/180.0f);
//			double x = 6*sin(3.0f*PI/180.0f);
//            double z =6*cos(3.0f*PI/180.0f);
			legs[0].x = x_for_back_legs;
			legs[0].z = z_for_back_legs;
			legs[1].x = x_for_back_legs;
			legs[1].z = z_for_back_legs;
			legs[2].x = x__for_front_legs;
			legs[2].z = z__for_front_legs;
			legs[3].x = x__for_front_legs;
			legs[3].z = z__for_front_legs;
			CartesianToTheta_Cycloid_All_Legs();
			// 控制腿部运动
			Moveleg();
			Motor_Auto_Run();
			if(t>=prep_time+execute_time_1+recovery_time)
			{
				
				Reset_start_time();
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
            double x__for_front_legs = (6.0f*K+(1.0f-K)*StandHeight)*sin(K*(-20.0f)*PI/180.0f);
            double z__for_front_legs = (6.0f*K+(1.0f-K)*StandHeight)*cos(K*20.0f*PI/180.0f);
			double x_for_back_legs = (6.0f*K+(1.0f-K)*StandHeight)*sin(K*(-20.0f)*PI/180.0f);
            double z_for_back_legs = (6.0f*K+(1.0f-K)*StandHeight)*cos(K*20.0f*PI/180.0f);
		legs[0].x = x_for_back_legs;
			legs[0].z = z_for_back_legs;
			legs[1].x = x_for_back_legs;
			legs[1].z = z_for_back_legs;
			legs[2].x = x__for_front_legs;
			legs[2].z = z__for_front_legs;
			legs[3].x = x__for_front_legs;
			legs[3].z = z__for_front_legs;
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
				x__for_front_legs = 18.5f*sin(-25.0f*PI/180.0f);
				z__for_front_legs = 18.5f*cos(25.0f*PI/180.0f);
			}
			else
			{
				x__for_front_legs = 6*sin(1.0f*PI/180.0f);
				z__for_front_legs = 6*cos(1.0f*PI/180.0f);
			}
						
			double x_for_back_legs = 18.5f*sin(-25.0f*PI/180.0f);
			double z_for_back_legs = 18.5f*cos(25.0f*PI/180.0f);
			legs[0].x = x_for_back_legs;
			legs[0].z = z_for_back_legs;
			legs[1].x = x_for_back_legs;
			legs[1].z = z_for_back_legs;
			legs[2].x = x__for_front_legs;
			legs[2].z = z__for_front_legs;
			legs[3].x = x__for_front_legs;
			legs[3].z = z__for_front_legs;
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
			double x__for_front_legs = 6*sin(1.0f*PI/180.0f);
            double z__for_front_legs =6*cos(1.0f*PI/180.0f);
			double x_for_back_legs = 6*sin(-8.0f*PI/180.0f);
            double z_for_back_legs = 6*cos(8.0f*PI/180.0f);
		
			legs[0].x = x_for_back_legs;
			legs[0].z = z_for_back_legs;
			legs[1].x = x_for_back_legs;
			legs[1].z = z_for_back_legs;
			legs[2].x = x__for_front_legs;
			legs[2].z = z__for_front_legs;
			legs[3].x = x__for_front_legs;
			legs[3].z = z__for_front_legs;
			CartesianToTheta_Cycloid_All_Legs();
			// 控制腿部运动
			Moveleg();
			Motor_Auto_Run();
			if(t>=prep_time+execute_time_1+recovery_time)
			{
				
				Reset_start_time();
				Jump_OK = 0;	//完成跳跃置零
			}
			
			
		}
	}
}

void Reset_start_time(void) {
    start_time = HAL_GetTick();
}