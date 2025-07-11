#include "Jump.h"
#include "ges_cal.h"
#include "pid.h"
#include "motor.h"
#include "RC.h"
//from remotectrl_init.c
//extern int left_x, left_y, right_x, right_y; 



//��Ծʱ��
const float bend_time=200; //�����ʱ��  
const float lean_time=200; //��б��ʱ��
const float exe_jump_time=80
	; //����ִ��ʱ�� 40ms // 0.04 
const float fall_time=200; //��ػָ�ʱ�� 0.03 30ms
const float jump_time_offset=30
	; //ǰ������Ծƫ��ֵ ����С��exe_jump_time

//��Ծ����ȳ�����
const double stretch_length=28.0f; //��չ����
const double shrink_length=14.0f; //��������
const double fall_length=14.0f ;//��ػ��峤��


//=============��Ծ�Ƕ�����===============
double pre_front_legs_angle=0;
double pre_behind_legs_angle=0;

double exe_front_legs_angle=-3;
double exe_behind_legs_angle=-3;
//��ֵ
const double rcv_front_legs_angle=3;
const double rcv_behind_legs_angle=2;

//=============����Ծ׼������ر���================
static uint16_t bend_count=0; //�������ʱ��
static uint16_t lean_count=0; //��б�Ƕȼ���ʱ��
static uint16_t exe_jump_count=0; //��Ծ����ʱ��
//��־λ
uint8_t bend_flag=0; //��ɺ���һ
uint8_t lean_flag=0; //��ɺ���һ
uint8_t Jump_OK=1;

extern uint16_t rc_left_x,rc_left_y,rc_right_x,rc_right_y;
extern uint8_t pre_angle;
extern JumpState_t jump_state;
extern CtrlState_t ctrl_state;
//================��Ծ��غ���====================
void Bend(void) //�����ȸ���
{
	double x_front_legs=0;
	double z_front_legs=StandHeight;
	double x_behind_legs=0;
	double z_behind_legs=StandHeight;
	ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
	Set_Max_Output_SL(8000);
	Set_Max_Output_PL(8000);
//	double K =(float)bend_count/(float)bend_time;
//	if(K>1)K=1;
//	x_front_legs = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*0*PI/180.0f);
//	z_front_legs = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*0*PI/180.0f);
//	x_behind_legs = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*0*PI/180.0f);
//	z_behind_legs = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*0*PI/180.0f);	

	x_front_legs = (shrink_length)*sin(0*PI/180.0f);
	z_front_legs = (shrink_length)*cos(0*PI/180.0f);
	x_behind_legs = (shrink_length)*sin(0*PI/180.0f);
	z_behind_legs = (shrink_length)*cos(0*PI/180.0f);	
	
	legs[2].x = x_behind_legs;
	legs[2].z = z_behind_legs;
	legs[3].x = x_behind_legs;
	legs[3].z = z_behind_legs;
	legs[0].x = x_front_legs;
	legs[0].z = z_front_legs;
	legs[1].x = x_front_legs;
	legs[1].z = z_front_legs;
	// ת�������˶�ѧ�ĽǶ�
	CartesianToTheta_Cycloid_All_Legs();
	// �����Ȳ��˶�
	Moveleg();
	Motor_Auto_Run();
	
	if(bend_count<=bend_time)
		bend_count++;
	if(bend_count>=bend_time)
		bend_flag=1;
}

void Lean(void) //0-30��//����׼���׶ε���б�Ƕ�
{
	
	pre_front_legs_angle=-(float)pre_angle;
	pre_behind_legs_angle=pre_front_legs_angle;
	
	exe_front_legs_angle=pre_front_legs_angle-3;
	exe_behind_legs_angle=pre_behind_legs_angle-3;
	
//=============================
	double x_front_legs=0;
	double z_front_legs=StandHeight;
	double x_behind_legs=0;
	double z_behind_legs=StandHeight;
	ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
	Set_Max_Output_SL(8000);
	Set_Max_Output_PL(8000);

//	double K =(float)lean_count/(float)lean_time;
//	if(K>1)K=1;
	
	x_front_legs=shrink_length*sin(1*pre_front_legs_angle*PI/180.0f);
	z_front_legs=shrink_length*cos(1*pre_front_legs_angle*PI/180.0f);
	x_behind_legs=shrink_length*sin(1*pre_behind_legs_angle*PI/180.0f);
	z_behind_legs=shrink_length*cos(1*pre_behind_legs_angle*PI/180.0f);	

	legs[2].x = x_behind_legs;
	legs[2].z = z_behind_legs;
	legs[3].x = x_behind_legs;
	legs[3].z = z_behind_legs;
	legs[0].x = x_front_legs;
	legs[0].z = z_front_legs;
	legs[1].x = x_front_legs;
	legs[1].z = z_front_legs;
	// ת�������˶�ѧ�ĽǶ�
	CartesianToTheta_Cycloid_All_Legs();
	// �����Ȳ��˶�
	Moveleg();
	Motor_Auto_Run();
	if(lean_count<=lean_time)
		lean_count++;
	if(lean_count>=lean_time)
		lean_flag=1;
}

void executeJump(void)
{
	exe_jump_count++;
	double x_front_legs=0;
	double z_front_legs=StandHeight;
	double x_behind_legs=0;
	double z_behind_legs=StandHeight;
	
	if ( exe_jump_count <exe_jump_time) 
	{
		ChangeTheGainOfPID_KP_KI_KD(8,0.3,1.1,22,0.3,5.5);
		Set_Max_Output_SL(12000);
		Set_Max_Output_PL(12000);
		if(exe_jump_count<exe_jump_time-jump_time_offset) 
		{
			ChangeTheGainOfPID_KP_KI_KD(8,0.3,1.1,22,0.3,5.5);
			Set_Max_Output_SL(12000);
			Set_Max_Output_PL(12000);
			x_front_legs = stretch_length*sin(exe_front_legs_angle*PI/180.0f); 
			z_front_legs = stretch_length*cos(exe_front_legs_angle*PI/180.0f);
		}
		else //ǰ��������
		{
			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			x_front_legs = fall_length*sin(rcv_front_legs_angle*PI/180.0f);
			z_front_legs = fall_length*cos(rcv_front_legs_angle*PI/180.0f);
		}
		
		x_behind_legs = stretch_length*sin(exe_behind_legs_angle*PI/180.0f);
		z_behind_legs = stretch_length*cos(exe_behind_legs_angle*PI/180.0f);
	}

	else 
	{
		ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
		Set_Max_Output_SL(8000);
		Set_Max_Output_PL(8000);
		x_front_legs = fall_length*sin(rcv_front_legs_angle*PI/180.0f);
		z_front_legs = fall_length*cos(rcv_front_legs_angle*PI/180.0f);
		x_behind_legs = fall_length*sin(rcv_behind_legs_angle*PI/180.0f);
		z_behind_legs = fall_length*cos(rcv_behind_legs_angle*PI/180.0f);
		if(exe_jump_count>=exe_jump_time+fall_time)
		{
			
			exe_jump_count=0; //��Ծʱ������
			Jump_OK = 1;	//�����Ծ��һ
		}
	}
	
	legs[2].x = x_behind_legs;
	legs[2].z = z_behind_legs;
	legs[3].x = x_behind_legs;
	legs[3].z = z_behind_legs;
	legs[0].x = x_front_legs;
	legs[0].z = z_front_legs;
	legs[1].x = x_front_legs;
	legs[1].z = z_front_legs;
	CartesianToTheta_Cycloid_All_Legs();
	// �����Ȳ��˶�
	Moveleg();
	Motor_Auto_Run();
	
}
//  IDLE,	����
//	BEND,	���� step1
//	LEAN,	��б step2
//	EXE, 	ִ�� step3
void jumpCtrl(void)
{
	switch (jump_state)
	{
		case IDLE:
		//��ر�־λ����	
			Jump_OK=0;
			bend_flag=0;
			lean_flag=0;
		//����ʱ������
			exe_jump_count=0;
			lean_count=0;
			bend_count=0;
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			// ת�������˶�ѧ�ĽǶ�
			CartesianToTheta_Cycloid_All_Legs();
			// �����Ȳ��˶�
			Moveleg();
			Motor_Auto_Run();
			break;
		if(ctrl_state==CS_PRE_JUMP)
		{
			case BEND:
				if(bend_flag==0&&lean_flag==0)
				{
					Bend();
				}
				else if(bend_flag==1&&lean_flag==0)
				{
					Set_Max_Output_SL(8000);
					Set_Max_Output_PL(8000);
					// ת�������˶�ѧ�ĽǶ�
					CartesianToTheta_Cycloid_All_Legs();
					// �����Ȳ��˶�
					Moveleg();
					Motor_Auto_Run();
				}
				break;
			case LEAN:
				if(bend_flag==1&&lean_flag==0)
				{
					Lean();
				}
				else if(bend_flag==1&&lean_flag==1)
				{
					Set_Max_Output_SL(8000);
					Set_Max_Output_PL(8000);
					// ת�������˶�ѧ�ĽǶ�
					CartesianToTheta_Cycloid_All_Legs();
					// �����Ȳ��˶�
					Moveleg();
					Motor_Auto_Run();
				}
				break;
		}
		else if(ctrl_state==CS_EXE_JUMP)
		{
			case EXE:
				if(bend_flag==1&&lean_flag==1)
				{
					if(Jump_OK==0)
					{
						executeJump();
					}
					else
					{
						Set_Max_Output_SL(8000);
						Set_Max_Output_PL(8000);
						// ת�������˶�ѧ�ĽǶ�
						CartesianToTheta_Cycloid_All_Legs();
						// �����Ȳ��˶�
						Moveleg();
						Motor_Auto_Run();
					}
				}
				break;
		}
		
		default:
			break;
	}

}	


////prepare
////front
//const double pre_jump1_front_legs_rotate_angle=-12.0f;
//const double pre_jump1_behind_legs_rotate_angle=-15.0f;
////behind
//const double pre_jump2_front_legs_rotate_angle=-23.0f;
//const double pre_jump2_behind_legs_rotate_angle=-23.0f;
////execute
////front
//const double exe_jump1_front_legs_rotate_angle=-20.0f;
//const double exe_jump1_behind_legs_rotate_angle=-20.0f;
////behind
//const double exe_jump2_front_legs_rotate_angle=-28.0f;
//const double exe_jump2_behind_legs_rotate_angle=-28.0f;
////recovery
////front
//const double rcv_jump1_front_legs_rotate_angle=3.0f;
//const double rcv_jump1_behind_legs_rotate_angle=3.0f;
////behind
//const double rcv_jump2_front_legs_rotate_angle=1.0f;
//const double rcv_jump2_behind_legs_rotate_angle=2.0f;


//void Execute_Jump(void) //��ʱ������5msһ�ж�
//{
//    uint16_t t =Get_Jump_Time();;
//   
//	if(ctrl_state==CS_JUMP_1)
//	{
//			
//			if (t < prep_time) {
//			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
//			Set_Max_Output_SL(8000);
//			Set_Max_Output_PL(8000);
//			double K =t/prep_time;
//			if(K>1)K=1;
//			double x__for_front_legs = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*pre_jump1_front_legs_rotate_angle*PI/180.0f);
//			double z__for_front_legs = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*pre_jump1_front_legs_rotate_angle*PI/180.0f);
//			double x_for_back_legs = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*pre_jump1_behind_legs_rotate_angle*PI/180.0f);
//			double z_for_back_legs = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*pre_jump1_behind_legs_rotate_angle*PI/180.0f);	
//				legs[2].x = x_for_back_legs;
//				legs[2].z = z_for_back_legs;
//				legs[3].x = x_for_back_legs;
//				legs[3].z = z_for_back_legs;
//				legs[0].x = x__for_front_legs;
//				legs[0].z = z__for_front_legs;
//				legs[1].x = x__for_front_legs;
//				legs[1].z = z__for_front_legs;
//			// ת�������˶�ѧ�ĽǶ�
//			CartesianToTheta_Cycloid_All_Legs();
//			// �����Ȳ��˶�
//			Moveleg();
//			Motor_Auto_Run();

//			}
//			else if ( t < prep_time+execute_time_1) {
//				ChangeTheGainOfPID_KP_KI_KD(8,0.3,1.1,22,0.3,5.5);
//				Set_Max_Output_SL(12000);
//				Set_Max_Output_PL(12000);
//				double x__for_front_legs;
//				double z__for_front_legs;
//				if(t<prep_time+front_legs_execute_time)
//				{
//					 x__for_front_legs = stretch_length*sin(exe_jump1_front_legs_rotate_angle*PI/180.0f);
//					 z__for_front_legs = stretch_length*cos(exe_jump1_front_legs_rotate_angle*PI/180.0f);
//				}
//				else
//				{
//					 x__for_front_legs = shrink_length*sin(rcv_jump1_front_legs_rotate_angle*PI/180.0f);
//					 z__for_front_legs = shrink_length*cos(rcv_jump1_front_legs_rotate_angle*PI/180.0f);
//				}
//				
//				double x_for_back_legs = stretch_length*sin(exe_jump1_behind_legs_rotate_angle*PI/180.0f);
//				double z_for_back_legs = stretch_length*cos(exe_jump1_behind_legs_rotate_angle*PI/180.0f);
//				legs[2].x = x_for_back_legs;
//				legs[2].z = z_for_back_legs;
//				legs[3].x = x_for_back_legs;
//				legs[3].z = z_for_back_legs;
//				legs[0].x = x__for_front_legs;
//				legs[0].z = z__for_front_legs;
//				legs[1].x = x__for_front_legs;
//				legs[1].z = z__for_front_legs;
//				// ת�������˶�ѧ�ĽǶ�
//				CartesianToTheta_Cycloid_All_Legs();
//				// �����Ȳ��˶�
//				Moveleg();
//				Motor_Auto_Run();
//			}

//		else 
//		{
//			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
//			Set_Max_Output_SL(8000);
//			Set_Max_Output_PL(8000);
////			double K2=(t-(prep_time+execute_time_1+execute_time_2+fly_time))/(recovery_time);
//		
////			double x = (StandHeight*K2+(1.0f-K2)*6.0f)*sin(0.0f*PI/180.0f);
////            double z = (StandHeight*K2+(1.0f-K2)*6.0f)*cos(0.0f*PI/180.0f);
//			double x__for_front_legs = fall_length*sin(rcv_jump1_front_legs_rotate_angle*PI/180.0f);
//            double z__for_front_legs = fall_length*cos(rcv_jump1_front_legs_rotate_angle*PI/180.0f);
//			double x_for_back_legs = fall_length*sin(rcv_jump1_behind_legs_rotate_angle*PI/180.0f);
//            double z_for_back_legs = fall_length*cos(rcv_jump1_behind_legs_rotate_angle*PI/180.0f);
////			double x = 6*sin(3.0f*PI/180.0f);
////            double z =6*cos(3.0f*PI/180.0f);
//			legs[2].x = x_for_back_legs;
//			legs[2].z = z_for_back_legs;
//			legs[3].x = x_for_back_legs;
//			legs[3].z = z_for_back_legs;
//			legs[0].x = x__for_front_legs;
//			legs[0].z = z__for_front_legs;
//			legs[1].x = x__for_front_legs;
//			legs[1].z = z__for_front_legs;
//			CartesianToTheta_Cycloid_All_Legs();
//			// �����Ȳ��˶�
//			Moveleg();
//			Motor_Auto_Run();
//			if(t>=prep_time+execute_time_1+recovery_time)
//			{
//				
//				Reset_jump_time();
//				Jump_OK = 0;	//�����Ծ����
//			}
//			
//			
//		}
//	
//	}
//	else if (ctrl_state == CS_JUMP_2)
//	{
//		ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
//			Set_Max_Output_SL(8000);
//			Set_Max_Output_PL(8000);
//			if (t < prep_time) {
//			double K =t/prep_time;
//			if(K>1)K=1;
//            double x__for_front_legs = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*pre_jump2_front_legs_rotate_angle*PI/180.0f);
//            double z__for_front_legs = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*pre_jump2_front_legs_rotate_angle*PI/180.0f);
//			double x_for_back_legs = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*pre_jump2_behind_legs_rotate_angle*PI/180.0f);
//            double z_for_back_legs = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*pre_jump2_behind_legs_rotate_angle*PI/180.0f);
//			legs[2].x = x_for_back_legs;
//			legs[2].z = z_for_back_legs;
//			legs[3].x = x_for_back_legs;
//			legs[3].z = z_for_back_legs;
//			legs[0].x = x__for_front_legs;
//			legs[0].z = z__for_front_legs;
//			legs[1].x = x__for_front_legs;
//			legs[1].z = z__for_front_legs;
//			// ת�������˶�ѧ�ĽǶ�
//			CartesianToTheta_Cycloid_All_Legs();
//			// �����Ȳ��˶�
//			Moveleg();
//			Motor_Auto_Run();
//           //printf("waiting\n");
//        }
//        else if ( t < prep_time+execute_time_1) {
//			ChangeTheGainOfPID_KP_KI_KD(8,0.3,1.1,22,0.3,2.5);
////			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
//			Set_Max_Output_SL(12000);
//			Set_Max_Output_PL(12000);
//			
//			double x__for_front_legs;
//			double z__for_front_legs;
//			if(t<prep_time+front_legs_execute_time)
//			{
//				x__for_front_legs = stretch_length*sin(exe_jump2_front_legs_rotate_angle*PI/180.0f);
//				z__for_front_legs = stretch_length*cos(exe_jump2_front_legs_rotate_angle*PI/180.0f);
//			}
//			else
//			{
//				x__for_front_legs = fall_length*sin(rcv_jump2_front_legs_rotate_angle*PI/180.0f);
//				z__for_front_legs = fall_length*cos(rcv_jump2_front_legs_rotate_angle*PI/180.0f);
//			}
//						
//			double x_for_back_legs = stretch_length*sin(exe_jump2_behind_legs_rotate_angle*PI/180.0f);
//			double z_for_back_legs = stretch_length*cos(exe_jump2_behind_legs_rotate_angle*PI/180.0f);
//			legs[2].x = x_for_back_legs;
//			legs[2].z = z_for_back_legs;
//			legs[3].x = x_for_back_legs;
//			legs[3].z = z_for_back_legs;
//			legs[0].x = x__for_front_legs;
//			legs[0].z = z__for_front_legs;
//			legs[1].x = x__for_front_legs;
//			legs[1].z = z__for_front_legs;
//			CartesianToTheta_Cycloid_All_Legs();
//			// �����Ȳ��˶�
//			Moveleg();
//			Motor_Auto_Run();
//			
//        }

//		else 
//		{
//			ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
//			Set_Max_Output_SL(8000);
//			Set_Max_Output_PL(8000);
//			double x__for_front_legs = fall_length*sin(rcv_jump2_front_legs_rotate_angle*PI/180.0f);
//            double z__for_front_legs =fall_length*cos(rcv_jump2_front_legs_rotate_angle*PI/180.0f);
//			double x_for_back_legs = fall_length*sin(rcv_jump2_behind_legs_rotate_angle*PI/180.0f);
//            double z_for_back_legs = fall_length*cos(rcv_jump2_behind_legs_rotate_angle*PI/180.0f);
//		
//			legs[2].x = x_for_back_legs;
//			legs[2].z = z_for_back_legs;
//			legs[3].x = x_for_back_legs;
//			legs[3].z = z_for_back_legs;
//			legs[0].x = x__for_front_legs;
//			legs[0].z = z__for_front_legs;
//			legs[1].x = x__for_front_legs;
//			legs[1].z = z__for_front_legs;
//			CartesianToTheta_Cycloid_All_Legs();
//			// �����Ȳ��˶�
//			Moveleg();
//			Motor_Auto_Run();
//			if(t>=prep_time+execute_time_1+recovery_time)
//			{
//				
//				Reset_jump_time();
//				Jump_OK = 0;	//�����Ծ����
//			}
//			
//			
//		}
//	}
//}

