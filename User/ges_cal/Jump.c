#include "Jump.h"
#include "ges_cal.h"
#include "pid.h"
#include "motor.h"
#include "RC.h"
#include "GaitParams.h"
#include "../DEFINE/define_file.h"
#include "math.h"
#define STRETCH_LENGTH 32.0f
#define SHRINK_LENGTH 12.0f
#define FALL_LENGTH 16.0f
#define TIME_OFFSET 0.15f //0.15s 前后腿动作时间差

extern uint16_t rc_left_x,rc_left_y,rc_right_x,rc_right_y;
extern uint8_t pre_angle;
extern JumpState_t jump_state;
extern CtrlState_t ctrl_state;
extern Leg legs[4]; //legs[2],legs[3]是后腿，legs[0],legs[1]是前腿
extern GaitParams  gait_params[][4];

JumpParameter_t jump1_struct;
JumpParameter_t jump2_struct;


void jumpInit(JumpParameter_t *js,byte index)
{
	if(index==1) //jump1
	{
		ZERO(js); //初始化为零
		REP(i,4)
		{
			// js->bend_struct[i].flag = 0;
			// js->bend_struct[i].rotate_count = 0;
			// js->bend_struct[i].stretch_count = 0;
			// js->bend_struct[i].rotate_prev_t = 0;
			// js->bend_struct[i].stretch_prev_t = 0;
			// js->bend_struct[i].rotate_angle = 0.0f; //进入Bend函数时设置
			js->bend_struct[i].stretch_length = SHRINK_LENGTH;
			js->bend_struct[i].rotate_freq = 1;
			js->bend_struct[i].stretch_freq = 1;
			js->bend_struct[i].rotate_time = 0.5;
			js->bend_struct[i].stretch_time = 0.5;

			
			// js->lean_struct[i].flag = 0;
			// js->lean_struct[i].rotate_count = 0;
			// js->lean_struct[i].stretch_count = 0;
			// js->lean_struct[i].rotate_prev_t = 0;
			// js->lean_struct[i].stretch_prev_t = 0;
			// js->lean_struct[i].rotate_angle = 0.0f; //进入Lean函数时设置
			js->lean_struct[i].rotate_freq = 1;
			js->lean_struct[i].stretch_freq = 1;
			js->lean_struct[i].rotate_time = 0.5;
			js->lean_struct[i].stretch_time = 0.5;
			js->lean_struct[i].stretch_length = SHRINK_LENGTH;

			//bend_lean
			js->bend_lean_struct[i].rotate_freq = 1;
			js->bend_lean_struct[i].stretch_freq = 1;
			js->bend_lean_struct[i].rotate_time = 0.5;
			js->bend_lean_struct[i].stretch_time = 0.5;
			js->bend_lean_struct[i].rotate_angle=
			js->bend_lean_struct[i].stretch_length = SHRINK_LENGTH;
			// js->bend_lean_struct[i].rotate_angle = 0.0f; //进入Lean函数时设置


			// js->exe_jump_struct[i].flag = 0;
			// js->exe_jump_struct[i].rotate_count = 0;
			// js->exe_jump_struct[i].stretch_count = 0;
			// js->exe_jump_struct[i].rotate_prev_t = 0;
			// js->exe_jump_struct[i].stretch_prev_t = 0;
			// js->exe_jump_struct[i].rotate_angle = 0.0f; //进入exeJump函数时设置
			js->exe_jump_struct[i].stretch_length = STRETCH_LENGTH;
			js->exe_jump_struct[i].rotate_freq = 10.0;
			js->exe_jump_struct[i].stretch_freq = 12.0;
			js->exe_jump_struct[i].rotate_time = 0.8;
			js->exe_jump_struct[i].stretch_time = 0.8;

			// js->rcv_jump_struct[i].flag = 0;
			// js->rcv_jump_struct[i].rotate_count = 0;
			// js->rcv_jump_struct[i].stretch_count = 0;
			// js->rcv_jump_struct[i].rotate_prev_t = 0;
			// js->rcv_jump_struct[i].stretch_prev_t = 0;

			js->rcv_jump_struct[i].rotate_freq = 5;
			js->rcv_jump_struct[i].stretch_freq = 5;
			js->rcv_jump_struct[i].rotate_time =0.5;
			js->rcv_jump_struct[i].stretch_time = 0.5;
			js->rcv_jump_struct[i].stretch_length = FALL_LENGTH;
			js->rcv_jump_struct[i].rotate_angle = 3.0f; //进入rcvJump函数时设置

		}
		// js->Jump_OK = 0; //一切跳跃完成后置1
		// js->Bend_Flag=0;
		// js->Lean_Flag=0;
	}
	else if (index==2) //jump2
	{
		ZERO(js); //初始化为零
		REP(i,2) //前腿
		{
			//用来准备阶段二者做准备
			//bend_lean
			js->bend_lean_struct[i].rotate_freq = 1;
			js->bend_lean_struct[i].stretch_freq = 1;
			js->bend_lean_struct[i].rotate_time = 0.5;
			js->bend_lean_struct[i].stretch_time = 0.5;
			js->bend_lean_struct[i].rotate_angle=-18.0f;
			js->bend_lean_struct[i].stretch_length = SHRINK_LENGTH;

			js->exe_jump_struct[i].rotate_angle = -15.0f;
			js->exe_jump_struct[i].stretch_length = STRETCH_LENGTH;
			js->exe_jump_struct[i].rotate_freq = 14.0;
			js->exe_jump_struct[i].stretch_freq = 14.0;
			js->exe_jump_struct[i].rotate_time = 0.8;
			js->exe_jump_struct[i].stretch_time = 0.8;

			js->rcv_jump_struct[i].rotate_freq = 5;
			js->rcv_jump_struct[i].stretch_freq = 5;
			js->rcv_jump_struct[i].rotate_time =0.5;
			js->rcv_jump_struct[i].stretch_time = 0.5;
			js->rcv_jump_struct[i].stretch_length = FALL_LENGTH;
			js->rcv_jump_struct[i].rotate_angle = 3.0f; //进入rcvJump函数时设置

		}
		FOR(i,2,4) //后腿
		{
			//用来给后腿在前腿跳跃的时候旋转
			js->lean_struct[i].rotate_angle =-60.0f; 
			js->lean_struct[i].rotate_freq = 8;
			js->lean_struct[i].stretch_freq = 8;
			js->lean_struct[i].rotate_time = 0.8; //0.8+0.5+0.2 exe_time+rcv_time+offset_time offset_time是延时的偏移值
			js->lean_struct[i].stretch_time = 0.8;
			js->lean_struct[i].stretch_length = SHRINK_LENGTH;

			//用来准备阶段二者做准备
			//bend_lean
			js->bend_lean_struct[i].rotate_freq = 1;
			js->bend_lean_struct[i].stretch_freq = 1;
			js->bend_lean_struct[i].rotate_time = 0.5;
			js->bend_lean_struct[i].stretch_time = 0.5;
			js->bend_lean_struct[i].rotate_angle=-15.0f;
			js->bend_lean_struct[i].stretch_length = SHRINK_LENGTH;
			// js->bend_lean_struct[i].rotate_angle = 0.0f; //进入Lean函数时设置

			js->exe_jump_struct[i].rotate_angle = js->lean_struct[i].rotate_angle; 
			js->exe_jump_struct[i].stretch_length = 35.0f;
			js->exe_jump_struct[i].rotate_freq = 14.0;
			js->exe_jump_struct[i].stretch_freq = 14.0;
			js->exe_jump_struct[i].rotate_time = 0.8;
			js->exe_jump_struct[i].stretch_time = 0.8;

			js->rcv_jump_struct[i].rotate_freq = 5;
			js->rcv_jump_struct[i].stretch_freq = 5;
			js->rcv_jump_struct[i].rotate_time =0.5;
			js->rcv_jump_struct[i].stretch_time = 0.5;
			js->rcv_jump_struct[i].stretch_length = FALL_LENGTH;
			js->rcv_jump_struct[i].rotate_angle = 3.0f; //进入rcvJump函数时设置

		}
	}
	
}

void resetJumpStructure(float t, JumpParameter_t *jump_structure)
{
	REP(i,4)
	{
		jump_structure->bend_struct[i].rotate_count = 0;
		jump_structure->bend_struct[i].stretch_count = 0;
		jump_structure->bend_struct[i].rotate_prev_t[0] = 0;
		jump_structure->bend_struct[i].stretch_prev_t[0] = 0;
		jump_structure->bend_struct[i].rotate_prev_t[1] = 0;
		jump_structure->bend_struct[i].stretch_prev_t[1] = 0;
		jump_structure->bend_struct[i].flag = 0;

		jump_structure->lean_struct[i].rotate_count = 0;
		jump_structure->lean_struct[i].stretch_count = 0;
		jump_structure->lean_struct[i].rotate_prev_t[0]  = 0;
		jump_structure->lean_struct[i].stretch_prev_t[0]  = 0;
		jump_structure->lean_struct[i].rotate_prev_t[1]  = 0;
		jump_structure->lean_struct[i].stretch_prev_t[1]  = 0;
		jump_structure->lean_struct[i].flag = 0;

		jump_structure->bend_lean_struct[i].rotate_count = 0;
		jump_structure->bend_lean_struct[i].stretch_count = 0;
		jump_structure->bend_lean_struct[i].rotate_prev_t[0]  = 0;
		jump_structure->bend_lean_struct[i].stretch_prev_t[0]  = 0;
		jump_structure->bend_lean_struct[i].rotate_prev_t[1]  = 0;
		jump_structure->bend_lean_struct[i].stretch_prev_t[1]  = 0;
		jump_structure->bend_lean_struct[i].flag = 0;

		jump_structure->exe_jump_struct[i].rotate_count = 0;
		jump_structure->exe_jump_struct[i].stretch_count = 0;
		jump_structure->exe_jump_struct[i].rotate_prev_t[0]  = 0;
		jump_structure->exe_jump_struct[i].stretch_prev_t[0]  = 0;
		jump_structure->exe_jump_struct[i].rotate_prev_t[1]  = 0;
		jump_structure->exe_jump_struct[i].stretch_prev_t[1]  = 0;
		jump_structure->exe_jump_struct[i].flag = 0;

		jump_structure->rcv_jump_struct[i].rotate_count = 0;
		jump_structure->rcv_jump_struct[i].stretch_count = 0;
		jump_structure->rcv_jump_struct[i].rotate_prev_t[0]  = 0;
		jump_structure->rcv_jump_struct[i].stretch_prev_t[0]  = 0;
		jump_structure->rcv_jump_struct[i].rotate_prev_t[1]  = 0;
		jump_structure->rcv_jump_struct[i].stretch_prev_t[1]  = 0;
		jump_structure->rcv_jump_struct[i].flag = 0;
	}
	jump_structure->Jump_OK = 0; //一切跳跃完成后置1
	jump_structure->Bend_Flag=0;
	jump_structure->Lean_Flag=0;
	//ZERO(jump_structure); //reset为零
}

byte Bend(float t, JumpParameter_t *js)
{
	REP(i,4)
	{
		js->bend_struct[i].flag=rotateAndStretch(t,js->bend_struct[i].rotate_angle
												,js->bend_struct[i].stretch_length,
												gait_params[0][i].stanceheight, //站立高度
												0,
												&legs[i],
												&js->bend_struct[i]);
	}
	if(js->bend_struct[0].flag
	&&js->bend_struct[1].flag
	&&js->bend_struct[2].flag
	&&js->bend_struct[3].flag) 
	{
		return 1; //bend完成
	}
	return 0;
}
byte Lean(float t, JumpParameter_t *js)
{
	REP(i,2)
	{
		js->lean_struct[i].rotate_angle=-(float)pre_angle;
		js->lean_struct[i].flag=rotateAndStretch(t,js->lean_struct[i].rotate_angle
												,js->lean_struct[i].stretch_length,
												js->bend_struct[i].stretch_length, //bend后高度
												0,			
												&legs[i],
												&js->lean_struct[i]);
		js->lean_struct[i+2].rotate_angle=-(float)pre_angle;
		js->lean_struct[i+2].flag=rotateAndStretch(t,js->lean_struct[i+2].rotate_angle
												,js->lean_struct[i+2].stretch_length,
												js->bend_struct[i+2].stretch_length, //bend后高度
												0,			
												&legs[i+2],
												&js->lean_struct[i+2]);
	}
	if(js->lean_struct[0].flag
	&&js->lean_struct[1].flag
	&&js->lean_struct[2].flag
	&&js->lean_struct[3].flag) 
	{
		return 1; //lean完成
	}
	return 0;
}

byte bendAndLean(float t, JumpParameter_t *js)
{
	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
	Set_Max_Output_SL(8000);
	Set_Max_Output_PL(8000);	
	REP(i,2)
	{
		js->bend_lean_struct[i].rotate_angle=-(float)pre_angle;
		js->bend_lean_struct[i].flag=rotateAndStretch(t,js->bend_lean_struct[i].rotate_angle
												,js->bend_lean_struct[i].stretch_length,
												StandHeight, //原高度
												0,			//原角度
												&legs[i],
												&js->bend_lean_struct[i]);
		js->bend_lean_struct[i+2].rotate_angle=-(float)pre_angle;
		js->bend_lean_struct[i+2].flag=rotateAndStretch(t,js->bend_lean_struct[i+2].rotate_angle //rotate angle
												,js->bend_lean_struct[i+2].stretch_length, //stretch length 
												StandHeight, 
												0,			//原角度
												&legs[i+2],
												&js->bend_lean_struct[i+2]);
	}
	if(js->bend_lean_struct[0].flag
	&&js->bend_lean_struct[1].flag
	&&js->bend_lean_struct[2].flag
	&&js->bend_lean_struct[3].flag) 
	{
		return 1; //lean完成
	}
	return 0;
}
byte exeJump(float t, JumpParameter_t *js)
{
	//ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
	ChangeTheGainOfPID_KP_KI_KD(12,0.1,0.01,25,0.01,1.5);
	REP(i,2)
	{
		js->exe_jump_struct[i].rotate_angle=js->bend_lean_struct[i].rotate_angle-3;	
		js->exe_jump_struct[i].flag=rotateAndStretch(t,js->exe_jump_struct[i].rotate_angle
												,js->exe_jump_struct[i].stretch_length,
												js->bend_lean_struct[i].stretch_length, //伸展高度
												js->bend_lean_struct[i].rotate_angle,
												&legs[i],
												&js->exe_jump_struct[i]);
		Set_Max_Output_SL(16384);
		Set_Max_Output_PL(16384);	
		//单独驱动前腿
		CartesianToTheta_Cycloid(&legs[i]);
		Angle_Setting_Cycloid(i);
		runSingleLeg(i);

		js->exe_jump_struct[i+2].rotate_angle=js->bend_lean_struct[i+2].rotate_angle-3;	
		//后腿跳跃蹬腿时期
		js->exe_jump_struct[i+2].flag=rotateAndStretch(t,js->exe_jump_struct[i+2].rotate_angle
												,js->exe_jump_struct[i+2].stretch_length,
												js->bend_lean_struct[i+2].stretch_length, //伸展高度
												js->bend_lean_struct[i+2].rotate_angle,
												&legs[i+2],
												&js->exe_jump_struct[i+2]);
		Set_Max_Output_SL(16384);
		Set_Max_Output_PL(16384);	
		//单独驱动前腿
		CartesianToTheta_Cycloid(&legs[i+2]);
		Angle_Setting_Cycloid(i+2);
		runSingleLeg(i+2);
		
	}
	if(js->exe_jump_struct[0].flag
		&&js->exe_jump_struct[1].flag
		&&js->exe_jump_struct[2].flag
		&&js->exe_jump_struct[3].flag) //前后腿都落地了
	{
		return 1; //完成
	}
	return 0;

}
byte rcvJump(float t,JumpParameter_t *js)
{
	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
	REP(i,2)
	{
		//前腿落地恢复时期
		js->rcv_jump_struct[i].flag=rotateAndStretch(t,js->rcv_jump_struct[i].rotate_angle
											,js->rcv_jump_struct[i].stretch_length,
											js->exe_jump_struct[i].stretch_length, //伸展高度
											js->exe_jump_struct[i].rotate_angle,
											&legs[i],
											&js->rcv_jump_struct[i]);
		Set_Max_Output_SL(8000);
		Set_Max_Output_PL(8000);	
		//单独驱动前腿
		CartesianToTheta_Cycloid(&legs[i]);
		Angle_Setting_Cycloid(i);
		runSingleLeg(i);

		js->rcv_jump_struct[i+2].flag=rotateAndStretch(t,js->rcv_jump_struct[i+2].rotate_angle
											,js->rcv_jump_struct[i+2].stretch_length,
											js->exe_jump_struct[i+2].stretch_length, //伸展高度
											js->exe_jump_struct[i+2].rotate_angle,
											&legs[i+2],
											&js->rcv_jump_struct[i+2]);
		Set_Max_Output_SL(8000);
		Set_Max_Output_PL(8000);	
		//单独驱动前腿
		CartesianToTheta_Cycloid(&legs[i+2]);
		Angle_Setting_Cycloid(i+2);
		runSingleLeg(i+2);
	}
	if(js->rcv_jump_struct[0].flag
		&&js->rcv_jump_struct[1].flag
		&&js->rcv_jump_struct[2].flag
		&&js->rcv_jump_struct[3].flag) //前后腿都落地了
	{
		return 1; //完成
	}
	return 0;
}
byte leanForSingleLeg(float t, JumpParameter_t *js , byte i,float rotate_angle,float stretch_length,float origin_angle,float origin_length)
{
	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
	Set_Max_Output_SL(8000);
	Set_Max_Output_PL(8000);	
	js->lean_struct[i].flag=rotateAndStretch(t,rotate_angle
											,stretch_length,
											origin_length, //bend后高度
											origin_angle,			
											&legs[i],
											&js->lean_struct[i]);
	
	if(js->lean_struct[i].flag)
	{
		return 1; //lean完成
	}
	return 0;
}
byte bendAndLeanForSingleLeg(float t, JumpParameter_t *js , byte i,float rotate_angle,float stretch_length,float origin_angle,float origin_length)
{
	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
	Set_Max_Output_SL(8000);
	Set_Max_Output_PL(8000);	
	js->bend_lean_struct[i].flag=rotateAndStretch(t,rotate_angle,
												stretch_length,
												origin_length, //原高度
												origin_angle,			//原角度
												&legs[i],
												&js->bend_lean_struct[i]);
		
	if(js->bend_lean_struct[i].flag) 
	{
		return 1; //lean完成
	}
	return 0;
}
byte jump2PreJump(float t, JumpParameter_t *js , float rotate_angle,float stretch_length,float origin_angle,float origin_length)
{
	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
	Set_Max_Output_SL(8000);
	Set_Max_Output_PL(8000);	
	REP(i,4)
	{
		js->bend_lean_struct[i].flag=rotateAndStretch(t,rotate_angle,
												stretch_length,
												origin_length, //原高度
												origin_angle,			//原角度
												&legs[i],
												&js->bend_lean_struct[i]);
	}
	if(js->bend_lean_struct[0].flag
		&&js->bend_lean_struct[1].flag
		&&js->bend_lean_struct[2].flag
		&&js->bend_lean_struct[3].flag) //前后腿都落地了
	{
		return 1; //完成
	}
	return 0;
}
byte exeJumpForSingleLeg(float t, JumpParameter_t *js , byte i,float rotate_angle,float stretch_length,float origin_angle,float origin_length)
{
	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
	Set_Max_Output_SL(8000);
	Set_Max_Output_PL(8000);	
	js->exe_jump_struct[i].flag=rotateAndStretch(t,rotate_angle,
												stretch_length,
												origin_length, //原高度
												origin_angle,			//原角度
												&legs[i],
												&js->exe_jump_struct[i]);
		
	if(js->exe_jump_struct[i].flag) 
	{
		return 1; //lean完成
	}
	return 0;
}
byte rcvJumpForSingleLeg(float t, JumpParameter_t *js , byte i,float rotate_angle,float stretch_length,float origin_angle,float origin_length)
{
	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
	Set_Max_Output_SL(8000);
	Set_Max_Output_PL(8000);	
	js->rcv_jump_struct[i].flag=rotateAndStretch(t,rotate_angle,
												stretch_length,
												origin_length, //原高度
												origin_angle,			//原角度
												&legs[i],
												&js->rcv_jump_struct[i]);
		
	if(js->rcv_jump_struct[i].flag) 
	{
		return 1; //lean完成
	}
	return 0;
}
//jump1
byte jump1_exe_flag=0;
byte jump1_rcv_flag=0;
byte jump1_bend_lean_flag=0;
//jump1
uint16_t jump1_count[2]={0};


//jump2
byte jump2_exe_flag[2]={0}; //分别为前腿，后腿的标志量
byte jump2_rcv_flag[2]={0};
byte jump2_bend_lean_flag[2]={0}; //分别为准备阶段所有腿，和前腿跳跃阶段后腿旋转时的标志量
//jump2
uint16_t jump2_count[5]={0}; //分别为前腿和后腿进行exe和rcv前的延时以及后腿旋转时的count front 0,1 behind 2,3 4

void setAngle(void) //set all legs move with the variation of pre_angle
{
	REP(i,4)
	{
		legs[i].x =gait_params[0][i].stanceheight * sin(-pre_angle* PI / 180.0f);
		legs[i].z =gait_params[0][i].stanceheight* cos( -pre_angle* PI / 180.0f);
	}
}
void jumpCtrl(float t,JumpParameter_t *js)
{
	switch (jump_state)
	{
		case IDLE:
			resetJumpStructure(t,js);
			//jump1
			jump1_exe_flag=0;
			jump1_rcv_flag=0;
			jump1_bend_lean_flag=0;
			ZERO(jump1_count); //reset count
			//jump2
			ZERO(jump2_exe_flag);
			ZERO(jump2_rcv_flag);
			ZERO(jump2_bend_lean_flag);
			ZERO(jump2_count);

			ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			CartesianToTheta_Cycloid_All_Legs();
			Moveleg();
			Motor_Auto_Run();
			break;
		case ANGLE_SET:
			setAngle();
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			CartesianToTheta_Cycloid_All_Legs();
			Moveleg();
			Motor_Auto_Run();
			break;
		case JUMP1:
			if(!jump1_bend_lean_flag)
			{
				ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
				Set_Max_Output_SL(8000);
				Set_Max_Output_PL(8000);
				jump1_bend_lean_flag=bendAndLean(t,js);
				CartesianToTheta_Cycloid_All_Legs();
				Moveleg();
				Motor_Auto_Run();
			}
			else if(!jump1_exe_flag&&!jump1_rcv_flag)
			{
				ChangeTheGainOfPID_KP_KI_KD(12,0.1,0.01,25,0.01,1.5);
				Set_Max_Output_SL(16384);
				Set_Max_Output_PL(16384);
				if(jump1_count[0]++>300)
				{
					jump1_exe_flag=exeJump(t,js);
					Can1_Send_Msg_to_Motor();
					Can2_Send_Msg_to_Motor();
				}
				else 
				{
					CartesianToTheta_Cycloid_All_Legs();
					Moveleg();
					Motor_Auto_Run();
				}
			}
			else if(!jump1_rcv_flag&&jump1_exe_flag)
			{
				ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
				Set_Max_Output_SL(8000);
				Set_Max_Output_PL(8000);
				if(jump1_count[1]++>50)
				{
					jump1_rcv_flag=rcvJump(t,js);
					Can1_Send_Msg_to_Motor();
					Can2_Send_Msg_to_Motor();
				}
				else 
				{
					CartesianToTheta_Cycloid_All_Legs();
					Moveleg();
					Motor_Auto_Run();
				}
					
			}
			else if(
				jump1_exe_flag
				&&
				jump1_rcv_flag
			)
			{
				ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
				Set_Max_Output_SL(8000);
				Set_Max_Output_PL(8000);
				CartesianToTheta_Cycloid_All_Legs();
				Moveleg();
				Motor_Auto_Run();
			}
				
			break;
		case JUMP2:
			if(!jump2_bend_lean_flag[0])
			{
				ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
				Set_Max_Output_SL(8000);
				Set_Max_Output_PL(8000);
				jump2_bend_lean_flag[0]=jump2PreJump(t,js,js->bend_lean_struct[0].rotate_angle,js->bend_lean_struct[0].stretch_length, 
										0,StandHeight);
				// bendAndLeanForSingleLeg(t,js,0,js->bend_lean_struct[0].rotate_angle,js->bend_lean_struct[0].stretch_length, //0
				// 						0,StandHeight)&&
				// 						bendAndLeanForSingleLeg(t,js,1,js->bend_lean_struct[1].rotate_angle,js->bend_lean_struct[1].stretch_length, //0
				// 						0,StandHeight)&&
				// 						bendAndLeanForSingleLeg(t,js,2,js->bend_lean_struct[2].rotate_angle,js->bend_lean_struct[2].stretch_length, //0
				// 						0,StandHeight)&&
				// 						bendAndLeanForSingleLeg(t,js,3,js->bend_lean_struct[3].rotate_angle,js->bend_lean_struct[3].stretch_length, //0
				// 						0,StandHeight);
				REP(i,4)
				{
					CartesianToTheta_Cycloid(&legs[i]);
					Angle_Setting_Cycloid(i);
					runSingleLeg(i);
				}
			}

			else if(
				jump2_bend_lean_flag[0]&&
				(!jump2_exe_flag[0]||
				!jump2_bend_lean_flag[1])
			)
			{
				changePIDForSingleLeg(12,0.1,0.01,25,0.01,1.5,0);
				changePIDForSingleLeg(12,0.1,0.01,25,0.01,1.5,1);
				setMaxSLForSingleLeg(16384,0); //前腿
				setMaxSLForSingleLeg(16384,1);
				if(jump2_count[0]++>50)
				{
					jump2_exe_flag[0]=exeJumpForSingleLeg(t,js,0,js->exe_jump_struct[0].rotate_angle,js->exe_jump_struct[0].stretch_length, //0
														js->bend_lean_struct[0].rotate_angle,js->bend_lean_struct[0].stretch_length)&&
									exeJumpForSingleLeg(t,js,1,js->exe_jump_struct[1].rotate_angle,js->exe_jump_struct[1].stretch_length, //1
														js->bend_lean_struct[1].rotate_angle,js->bend_lean_struct[1].stretch_length);
				}
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,2);
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,3);
				setMaxSLForSingleLeg(8000,0); //后腿
				setMaxSLForSingleLeg(8000,1);
				if(jump2_count[4]++>50)
				{
					jump2_bend_lean_flag[1]=leanForSingleLeg(t,js,2,js->lean_struct[2].rotate_angle,js->lean_struct[2].stretch_length,
															-js->bend_lean_struct[2].rotate_angle,js->bend_lean_struct[2].stretch_length)&& //两个后腿
											leanForSingleLeg(t,js,3,js->lean_struct[3].rotate_angle,js->lean_struct[3].stretch_length,
															js->bend_lean_struct[3].rotate_angle,js->bend_lean_struct[3].stretch_length);
				}
				REP(i,4)
				{
					CartesianToTheta_Cycloid(&legs[i]);
					Angle_Setting_Cycloid(i);
					runSingleLeg(i);
				}

			}
			else if(jump2_bend_lean_flag[0]&&
				(!jump2_rcv_flag[0]|| //前腿收缩
				!jump2_exe_flag[1])) //后腿伸腿
			{
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,0);
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,1);
				setMaxSLForSingleLeg(8000,0); //前腿
				setMaxSLForSingleLeg(8000,1);
				if(jump2_count[1]++>50) //rcv
				{
					jump2_rcv_flag[0]=rcvJumpForSingleLeg(t,js,0,js->rcv_jump_struct[0].rotate_angle,js->rcv_jump_struct[0].stretch_length, //0
														js->exe_jump_struct[0].rotate_angle,js->exe_jump_struct[0].stretch_length)&&
									rcvJumpForSingleLeg(t,js,1,js->rcv_jump_struct[1].rotate_angle,js->rcv_jump_struct[1].stretch_length, //1
														js->exe_jump_struct[1].rotate_angle,js->exe_jump_struct[1].stretch_length);
				}
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,2);
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,3);
				setMaxSLForSingleLeg(8000,2); //后腿
				setMaxSLForSingleLeg(8000,3);
				if(jump2_count[2]++>50)
				{
					jump2_exe_flag[1]=exeJumpForSingleLeg(t,js,2,js->exe_jump_struct[2].rotate_angle,js->exe_jump_struct[2].stretch_length,
														js->lean_struct[2].rotate_angle,js->lean_struct[2].stretch_length)&&
									exeJumpForSingleLeg(t,js,3,js->exe_jump_struct[3].rotate_angle,js->exe_jump_struct[3].stretch_length,
														js->lean_struct[3].rotate_angle,js->lean_struct[3].stretch_length);
				}
				REP(i,4)
				{
					CartesianToTheta_Cycloid(&legs[i]);
					Angle_Setting_Cycloid(i);
					runSingleLeg(i);
				}
			}
			else if(jump2_bend_lean_flag[0]&&
				(jump2_rcv_flag[0]|| //前腿收缩结束
				!jump2_rcv_flag[1])) //后腿收缩
			{
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,0);
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,1);
				setMaxSLForSingleLeg(8000,0); //前腿
				setMaxSLForSingleLeg(8000,1);
				//前腿不做变化
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,2);
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,3);
				setMaxSLForSingleLeg(8000,2); //后腿
				setMaxSLForSingleLeg(8000,3);
				if(jump2_count[3]++>50) //后腿 rcv的延时
				{
					jump2_rcv_flag[1]=rcvJumpForSingleLeg(t,js,2,js->rcv_jump_struct[2].rotate_angle,js->rcv_jump_struct[2].stretch_length,
														js->exe_jump_struct[2].rotate_angle,js->exe_jump_struct[2].stretch_length)&&
									rcvJumpForSingleLeg(t,js,3,js->rcv_jump_struct[2].rotate_angle,js->rcv_jump_struct[2].stretch_length,
														js->exe_jump_struct[2].rotate_angle,js->exe_jump_struct[2].stretch_length);
				}
				REP(i,4)
				{
					CartesianToTheta_Cycloid(&legs[i]);
					Angle_Setting_Cycloid(i);
					runSingleLeg(i);
				}
			}
			else if(jump2_bend_lean_flag[0]&&
				(jump2_rcv_flag[0]|| //前腿收缩结束
				jump2_rcv_flag[1])) //后腿收缩结束
			{
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,0);
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,1);
				setMaxSLForSingleLeg(8000,0); //前腿
				setMaxSLForSingleLeg(8000,1);
				//前腿不做变化
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,2);
				changePIDForSingleLeg(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D,3);
				setMaxSLForSingleLeg(8000,2); //后腿
				setMaxSLForSingleLeg(8000,3);
				//后腿不做变化
				REP(i,4)
				{
					CartesianToTheta_Cycloid(&legs[i]);
					Angle_Setting_Cycloid(i);
					runSingleLeg(i);
				}
			}

			// //front legs
			// if( //front legs exe jump 
			// 	(!jump2_exe_flag[0]&&!jump2_rcv_flag[0])&&
			// 	jump2_bend_lean_flag[0] //当准备动作完成后
			// 	)
			// {
			// 	ChangeTheGainOfPID_KP_KI_KD(12,0.1,0.01,25,0.01,1.5);
			// 	Set_Max_Output_SL(16384);
			// 	Set_Max_Output_PL(16384);
			// 	if(jump2_count[0]++>50)
			// 	{
			// 		jump2_exe_flag[0]=exeJumpForSingleLeg(t,js,0,js->exe_jump_struct[0].rotate_angle,js->exe_jump_struct[0].stretch_length, //0
			// 											js->bend_lean_struct[0].rotate_angle,js->bend_lean_struct[0].stretch_length)&&
			// 						exeJumpForSingleLeg(t,js,1,js->exe_jump_struct[1].rotate_angle,js->exe_jump_struct[1].stretch_length, //1
			// 											js->bend_lean_struct[1].rotate_angle,js->bend_lean_struct[1].stretch_length);
			// 	}
			// 	REP(i,2)
			// 	{
			// 		CartesianToTheta_Cycloid(&legs[i]);
			// 		Angle_Setting_Cycloid(i);
			// 		runSingleLeg(i);
			// 	}
			// }
			// else if ( //front legs rcv jump 
			// 	(jump2_exe_flag[0]&&!jump2_rcv_flag[0])&&
			// 	jump2_bend_lean_flag[0] //当准备动作完成后
			// 	)
			// {
			// 	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
			// 	Set_Max_Output_SL(8000);
			// 	Set_Max_Output_PL(8000);
			// 	if(jump2_count[1]++>50) //rcv
			// 	{
			// 		jump2_rcv_flag[0]=rcvJumpForSingleLeg(t,js,0,js->rcv_jump_struct[0].rotate_angle,js->rcv_jump_struct[0].stretch_length, //0
			// 											js->exe_jump_struct[0].rotate_angle,js->exe_jump_struct[0].stretch_length)&&
			// 						rcvJumpForSingleLeg(t,js,1,js->rcv_jump_struct[1].rotate_angle,js->rcv_jump_struct[1].stretch_length, //1
			// 											js->exe_jump_struct[1].rotate_angle,js->exe_jump_struct[1].stretch_length);
			// 	}
			// 	REP(i,2)
			// 	{
			// 		CartesianToTheta_Cycloid(&legs[i]);
			// 		Angle_Setting_Cycloid(i);
			// 		runSingleLeg(i);
			// 	}
			// }
			// else if( //front legs rcv jump 
			// 	(jump2_exe_flag[0]&&jump2_rcv_flag[0])&&
			// 	jump2_bend_lean_flag[0] //当准备动作完成后
			// 	)
			// {
			// 	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
			// 	Set_Max_Output_SL(8000);
			// 	Set_Max_Output_PL(8000);
			// 	REP(i,2)
			// 	{
			// 		CartesianToTheta_Cycloid(&legs[i]);
			// 		Angle_Setting_Cycloid(i);
			// 		runSingleLeg(i);
			// 	}
			// }

			// //behind legs
			// if(
			// 	(!jump2_exe_flag[1]&&!jump2_rcv_flag[1])&&
			// 	jump2_bend_lean_flag[0]&& //完成准备动作之后
			// 	!jump2_bend_lean_flag[1]
			// )
			// {
			// 	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
			// 	Set_Max_Output_SL(8000);
			// 	Set_Max_Output_PL(8000);
			// 	if(jump2_count[4]++>50)
			// 	{
			// 		jump2_bend_lean_flag[1]=leanForSingleLeg(t,js,2,js->lean_struct[2].rotate_angle,js->lean_struct[2].stretch_length,
			// 												-js->bend_lean_struct[2].rotate_angle,js->bend_lean_struct[2].stretch_length)&& //两个后腿
			// 								leanForSingleLeg(t,js,3,js->lean_struct[3].rotate_angle,js->lean_struct[3].stretch_length,
			// 												js->bend_lean_struct[3].rotate_angle,js->bend_lean_struct[3].stretch_length);
			// 	}
			// 	//单独驱动后腿
			// 	FOR(i,2,4)
			// 	{
			// 		CartesianToTheta_Cycloid(&legs[i]);
			// 		Angle_Setting_Cycloid(i);
			// 		runSingleLeg(i);
			// 	}
			// }
			// else if(
			// 	(!jump2_exe_flag[1]&&!jump2_rcv_flag[1])&&
			// 	jump2_bend_lean_flag[0]&& //完成准备动作之后
			// 	jump2_bend_lean_flag[1] //完成旋转动作之后
			// )
			// {
			// 	ChangeTheGainOfPID_KP_KI_KD(12,0.1,0.01,25,0.01,1.5);
			// 	Set_Max_Output_SL(16384);
			// 	Set_Max_Output_PL(16384);
			// 	if(jump2_count[2]++>50)
			// 	{
			// 		jump2_exe_flag[1]=exeJumpForSingleLeg(t,js,2,js->exe_jump_struct[2].rotate_angle,js->exe_jump_struct[2].stretch_length,
			// 											js->lean_struct[2].rotate_angle,js->lean_struct[2].stretch_length)&&
			// 						exeJumpForSingleLeg(t,js,3,js->exe_jump_struct[3].rotate_angle,js->exe_jump_struct[3].stretch_length,
			// 											js->lean_struct[3].rotate_angle,js->lean_struct[3].stretch_length);
			// 	}
			// 	FOR(i,2,4)
			// 	{
			// 		CartesianToTheta_Cycloid(&legs[i]);
			// 		Angle_Setting_Cycloid(i);
			// 		runSingleLeg(i);
			// 	}
			// }
			// else if(
			// 	(jump2_exe_flag[1]&&!jump2_rcv_flag[1])&& //exe执行完之后
			// 	jump2_bend_lean_flag[0]&& //完成准备动作之后
			// 	jump2_bend_lean_flag[1] //完成旋转动作之后
			// )
			// {
			// 	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
			// 	Set_Max_Output_SL(8000);
			// 	Set_Max_Output_PL(8000);
			// 	if(jump2_count[3]++>50) //后腿 rcv的延时
			// 	{
			// 		jump2_rcv_flag[1]=rcvJumpForSingleLeg(t,js,2,js->rcv_jump_struct[2].rotate_angle,js->rcv_jump_struct[2].stretch_length,
			// 											js->exe_jump_struct[2].rotate_angle,js->exe_jump_struct[2].stretch_length)&&
			// 						rcvJumpForSingleLeg(t,js,3,js->rcv_jump_struct[2].rotate_angle,js->rcv_jump_struct[2].stretch_length,
			// 											js->exe_jump_struct[2].rotate_angle,js->exe_jump_struct[2].stretch_length);
			// 	}
			// 	FOR(i,2,4)
			// 	{
			// 		CartesianToTheta_Cycloid(&legs[i]);
			// 		Angle_Setting_Cycloid(i);
			// 		runSingleLeg(i);
			// 	}
			// }
			// else if(
			// 	(jump2_exe_flag[1]&&jump2_rcv_flag[1])&& //exe rcv执行完之后
			// 	jump2_bend_lean_flag[0]&& //完成准备动作之后
			// 	jump2_bend_lean_flag[1] //完成旋转动作之后
			// )
			// {
			// 	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
			// 	Set_Max_Output_SL(8000);
			// 	Set_Max_Output_PL(8000);
			// 	FOR(i,2,4)
			// 	{
			// 		CartesianToTheta_Cycloid(&legs[i]);
			// 		Angle_Setting_Cycloid(i);
			// 		runSingleLeg(i);
			// 	}
			// }

			Can1_Send_Msg_to_Motor();
			Can2_Send_Msg_to_Motor();
			break;
		default:
			break;
	}
}