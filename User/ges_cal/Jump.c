#include "Jump.h"
#include "ges_cal.h"
#include "pid.h"
#include "motor.h"
#include "RC.h"
#include "GaitParams.h"
#include "../DEFINE/define_file.h"

#define STRETCH_LENGTH 30.0f
#define SHRINK_LENGTH 12.0f
#define FALL_LENGTH 14.0f
#define TIME_OFFSET 0.15f //0.15s 前后腿动作时间差

extern uint16_t rc_left_x,rc_left_y,rc_right_x,rc_right_y;
extern uint8_t pre_angle;
extern JumpState_t jump_state;
extern CtrlState_t ctrl_state;
extern Leg legs[4]; //legs[2],legs[3]是后腿，legs[0],legs[1]是前腿
extern GaitParams  gait_params[][4];

JumpParameter_t jump_structure;

void jumpInit(JumpParameter_t *js)
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
}

byte test1(float t, JumpParameter_t *js)
{
	REP(i,4)
	{
		js->exe_jump_struct[i].rotate_angle=js->lean_struct[i].rotate_angle-3;	
		js->exe_jump_struct[i].flag=rotateAndStretch(t,js->exe_jump_struct[i].rotate_angle
												,js->exe_jump_struct[i].stretch_length,
												js->lean_struct[i].stretch_length, //站立高度
												js->lean_struct[i].rotate_angle,
												&legs[i],
												&js->exe_jump_struct[i]);
	}
	if(js->exe_jump_struct[0].flag
	&&js->exe_jump_struct[1].flag
	&&js->exe_jump_struct[2].flag
	&&js->exe_jump_struct[3].flag) 
	{
		return 1; //bend完成
	}
	return 0;
}

byte test2(float t, JumpParameter_t *js)
{
	REP(i,4)
	{
		js->rcv_jump_struct[i].flag=rotateAndStretch(t,js->rcv_jump_struct[i].rotate_angle
												,js->rcv_jump_struct[i].stretch_length,
												js->exe_jump_struct[i].stretch_length, //站立高度
												js->exe_jump_struct[i].rotate_angle,
												&legs[i],
												&js->rcv_jump_struct[i]);
	}
	if(js->rcv_jump_struct[0].flag
	&&js->rcv_jump_struct[1].flag
	&&js->rcv_jump_struct[2].flag
	&&js->rcv_jump_struct[3].flag) 
	{
		return 1; //bend完成
	}
	return 0;
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

byte exeJump(float t, JumpParameter_t *js)
{
	//ChangeTheGainOfPID_KP_KI_KD(7.5,0.3,1.81,7.5,0.3,2.5);
	ChangeTheGainOfPID_KP_KI_KD(12,0.1,0.01,25,0.01,1.5);
	REP(i,2)
	{
		js->exe_jump_struct[i].rotate_angle=js->lean_struct[i].rotate_angle-3;	
		js->exe_jump_struct[i].flag=rotateAndStretch(t,js->exe_jump_struct[i].rotate_angle
												,js->exe_jump_struct[i].stretch_length,
												js->lean_struct[i].stretch_length, //伸展高度
												js->lean_struct[i].rotate_angle,
												&legs[i],
												&js->exe_jump_struct[i]);
		Set_Max_Output_SL(16384);
		Set_Max_Output_PL(16384);	
		//单独驱动前腿
		CartesianToTheta_Cycloid(&legs[i]);
		Angle_Setting_Cycloid(i);
		runSingleLeg(i);

		js->exe_jump_struct[i+2].rotate_angle=js->lean_struct[i+2].rotate_angle-3;	
		//后腿跳跃蹬腿时期
		js->exe_jump_struct[i+2].flag=rotateAndStretch(t,js->exe_jump_struct[i+2].rotate_angle
												,js->exe_jump_struct[i+2].stretch_length,
												js->lean_struct[i+2].stretch_length, //伸展高度
												js->lean_struct[i+2].rotate_angle,
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
byte exe_flag=0;
byte rcv_flag=0;
uint16_t count1=0;
void jumpCtrl(float t,JumpParameter_t *js)
{
	switch (jump_state)
	{
		case IDLE:
			resetJumpStructure(t,js);
			exe_flag=0;
			rcv_flag=0;
			count1=0;
			ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
			Set_Max_Output_SL(8000);
			Set_Max_Output_PL(8000);
			CartesianToTheta_Cycloid_All_Legs();
			Moveleg();
			Motor_Auto_Run();
			break;
		if(ctrl_state==CS_PRE_JUMP)
		{
			case BEND:
				if(!js->Bend_Flag)
				{
					js->Bend_Flag=Bend(t,js);
				}
				ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
				Set_Max_Output_SL(8000);
				Set_Max_Output_PL(8000);
				CartesianToTheta_Cycloid_All_Legs();
				Moveleg();
				Motor_Auto_Run();	
				break;
			case LEAN:
				if(!js->Lean_Flag)
				{
					js->Lean_Flag=Lean(t,js);
				}
				ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
				Set_Max_Output_SL(8000);
				Set_Max_Output_PL(8000);
				CartesianToTheta_Cycloid_All_Legs();
				Moveleg();
				Motor_Auto_Run();
		}
		else if(ctrl_state==CS_EXE_JUMP)
		{
			case EXE:
				if(!exe_flag&&!rcv_flag)
				{
					exe_flag=exeJump(t,js);
					Can1_Send_Msg_to_Motor();
					Can2_Send_Msg_to_Motor();
				}
				else if(!rcv_flag&&exe_flag)
				{
					if(count1++>50)
					{
						rcv_flag=rcvJump(t,js);
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
					exe_flag
					&&
					rcv_flag
				)
				{
					ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
					CartesianToTheta_Cycloid_All_Legs();
					Moveleg();
					Motor_Auto_Run();
				}
				
				break;
		}
		default:
			break;
	}
}