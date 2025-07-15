#include "Jump.h"
#include "pid.h"
#include "motor.h"
#include "RC.h"
#include "motion_ctrl.h"
#include "pid.h"



//跳跃时间
const float bend_time=200; //俯身基时间  
const float lean_time=200; //倾斜基时间
const float exe_jump_time=80; //蹬腿执行时间 40ms // 0.04 
const float fall_time=200; //落地恢复时间 0.03 30ms
const float jump_time_offset=30; //前后腿跳跃偏移值 必须小于exe_jump_time

//跳跃相关腿长参数
const float stretch_length=28.0f; //伸展长度
const float shrink_length=14.0f; //收缩长度
const float fall_length=14.0f ;//落地缓冲长度

//=============跳跃角度设置===============
float pre_front_legs_params_angle=0;
float pre_behind_legs_params_angle=0;

float exe_front_legs_params_angle=-3;
float exe_behind_legs_params_angle=-3;
//定值
const float rcv_front_legs_params_angle=3;
const float rcv_behind_legs_params_angle=2;

//=============“跳跃准备”相关变量================
static uint16_t bend_count=0; //俯身计数时间
static uint16_t lean_count=0; //倾斜角度计数时间
static uint16_t exe_jump_count=0; //跳跃计数时间
//标志位
uint8_t bend_flag=0; //完成后置一
uint8_t lean_flag=0; //完成后置一
uint8_t Jump_OK=1;

extern uint16_t rc_left_x,rc_left_y,rc_right_x,rc_right_y;
extern uint8_t pre_angle;
extern JumpState_t jump_state;
extern CtrlState_t ctrl_state;
extern LegParameter_t legs_params[4];
//================跳跃相关函数====================
void bend(void) //狗子先俯身
{
	float x_front_legs_params=0;
	float z_front_legs_params=StandHeight;
	float x_behind_legs_params=0;
	float z_behind_legs_params=StandHeight;
	setSpeedPosPid(7.5,0.3,1.81,7.5,0.3,2.5);
	setMaxSpeed(8000);
	setMaxPos(8000);
//	float K =(float)bend_count/(float)bend_time;
//	if(K>1)K=1;
//	x_front_legs_params = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*0*PI/180.0f);
//	z_front_legs_params = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*0*PI/180.0f);
//	x_behind_legs_params = (shrink_length*K+(1.0f-K)*StandHeight)*sin(K*0*PI/180.0f);
//	z_behind_legs_params = (shrink_length*K+(1.0f-K)*StandHeight)*cos(K*0*PI/180.0f);	

	x_front_legs_params = (shrink_length)*sin(0*PI/180.0f);
	z_front_legs_params = (shrink_length)*cos(0*PI/180.0f);
	x_behind_legs_params = (shrink_length)*sin(0*PI/180.0f);
	z_behind_legs_params = (shrink_length)*cos(0*PI/180.0f);	
	
	legs_params[2].x = x_behind_legs_params;
	legs_params[2].z = z_behind_legs_params;
	legs_params[3].x = x_behind_legs_params;
	legs_params[3].z = z_behind_legs_params;
	legs_params[0].x = x_front_legs_params;
	legs_params[0].z = z_front_legs_params;
	legs_params[1].x = x_front_legs_params;
	legs_params[1].z = z_front_legs_params;
    // 转换到逆运动学的角度
	transXYToAngle_SUM();
	// 传输各腿电机角度
	setMotionOutAngle();
	runMotor();
	
	if(bend_count<=bend_time)
		bend_count++;
	if(bend_count>=bend_time)
		bend_flag=1;
}

void lean(void) //0-30°//设置准备阶段的倾斜角度
{
	
	pre_front_legs_params_angle=-(float)pre_angle;
	pre_behind_legs_params_angle=pre_front_legs_params_angle;
	
	exe_front_legs_params_angle=pre_front_legs_params_angle-3;
	exe_behind_legs_params_angle=pre_behind_legs_params_angle-3;
	
//=============================
	float x_front_legs_params=0;
	float z_front_legs_params=StandHeight;
	float x_behind_legs_params=0;
	float z_behind_legs_params=StandHeight;
	setSpeedPosPid(7.5,0.3,1.81,7.5,0.3,2.5);
	setMaxSpeed(8000);
	setMaxPos(8000);

//	float K =(float)lean_count/(float)lean_time;
//	if(K>1)K=1;
	
	x_front_legs_params=shrink_length*sin(1*pre_front_legs_params_angle*PI/180.0f);
	z_front_legs_params=shrink_length*cos(1*pre_front_legs_params_angle*PI/180.0f);
	x_behind_legs_params=shrink_length*sin(1*pre_behind_legs_params_angle*PI/180.0f);
	z_behind_legs_params=shrink_length*cos(1*pre_behind_legs_params_angle*PI/180.0f);	

	legs_params[2].x = x_behind_legs_params;
	legs_params[2].z = z_behind_legs_params;
	legs_params[3].x = x_behind_legs_params;
	legs_params[3].z = z_behind_legs_params;
	legs_params[0].x = x_front_legs_params;
	legs_params[0].z = z_front_legs_params;
	legs_params[1].x = x_front_legs_params;
	legs_params[1].z = z_front_legs_params;
   // 转换到逆运动学的角度
	transXYToAngle_SUM();
	// 传输各腿电机角度
	setMotionOutAngle();
	runMotor();
	if(lean_count<=lean_time)
		lean_count++;
	if(lean_count>=lean_time)
		lean_flag=1;
}

void executeJump(void)
{
	exe_jump_count++;
	float x_front_legs_params=0;
	float z_front_legs_params=StandHeight;
	float x_behind_legs_params=0;
	float z_behind_legs_params=StandHeight;
	
	if ( exe_jump_count <exe_jump_time) 
	{
		setSpeedPosPid(8,0.3,1.1,22,0.3,5.5);
		setMaxSpeed(12000);
		setMaxPos(12000);
		if(exe_jump_count<exe_jump_time-jump_time_offset) 
		{
			setSpeedPosPid(8,0.3,1.1,22,0.3,5.5);
			setMaxSpeed(12000);
			setMaxPos(12000);
			x_front_legs_params = stretch_length*sin(exe_front_legs_params_angle*PI/180.0f); 
			z_front_legs_params = stretch_length*cos(exe_front_legs_params_angle*PI/180.0f);
		}
		else //前腿先收腿
		{
			setSpeedPosPid(7.5,0.3,1.81,7.5,0.3,2.5);
			setMaxSpeed(8000);
			setMaxPos(8000);
			x_front_legs_params = fall_length*sin(rcv_front_legs_params_angle*PI/180.0f);
			z_front_legs_params = fall_length*cos(rcv_front_legs_params_angle*PI/180.0f);
		}
		
		x_behind_legs_params = stretch_length*sin(exe_behind_legs_params_angle*PI/180.0f);
		z_behind_legs_params = stretch_length*cos(exe_behind_legs_params_angle*PI/180.0f);
	}

	else 
	{
		setSpeedPosPid(7.5,0.3,1.81,7.5,0.3,2.5);
		setMaxSpeed(8000);
		setMaxPos(8000);
		x_front_legs_params = fall_length*sin(rcv_front_legs_params_angle*PI/180.0f);
		z_front_legs_params = fall_length*cos(rcv_front_legs_params_angle*PI/180.0f);
		x_behind_legs_params = fall_length*sin(rcv_behind_legs_params_angle*PI/180.0f);
		z_behind_legs_params = fall_length*cos(rcv_behind_legs_params_angle*PI/180.0f);
		if(exe_jump_count>=exe_jump_time+fall_time)
		{
			
			exe_jump_count=0; //跳跃时间置零
			Jump_OK = 1;	//完成跳跃置一
		}
	}
	
	legs_params[2].x = x_behind_legs_params;
	legs_params[2].z = z_behind_legs_params;
	legs_params[3].x = x_behind_legs_params;
	legs_params[3].z = z_behind_legs_params;
	legs_params[0].x = x_front_legs_params;
	legs_params[0].z = z_front_legs_params;
	legs_params[1].x = x_front_legs_params;
	legs_params[1].z = z_front_legs_params;
	transXYToAngle_SUM();
	// 控制腿部运动
	setMotionOutAngle();
	runMotor();
	
}
//  IDLE,	空闲
//	BEND,	俯身 step1
//	LEAN,	倾斜 step2
//	EXE, 	执行 step3
void jumpCtrl(void)
{
	switch (jump_state)
	{
		case IDLE:
		//相关标志位置零	
			Jump_OK=0;
			bend_flag=0;
			lean_flag=0;
		//计数时间置零
			exe_jump_count=0;
			lean_count=0;
			bend_count=0;
			setMaxSpeed(8000);
			setMaxPos(8000);
			// 转换到逆运动学的角度
			transXYToAngle_SUM();
			// 控制腿部运动
			setMotionOutAngle();
			runMotor();
			break;
		if(ctrl_state==CS_PRE_JUMP)
		{
			case BEND:
				if(bend_flag==0&&lean_flag==0)
				{
					bend();
				}
				else if(bend_flag==1&&lean_flag==0)
				{
					setMaxSpeed(8000);
					setMaxPos(8000);
					// 转换到逆运动学的角度
					transXYToAngle_SUM();
					// 控制腿部运动
					setMotionOutAngle();
					runMotor();
				}
				break;
			case LEAN:
				if(bend_flag==1&&lean_flag==0)
				{
					lean();
				}
				else if(bend_flag==1&&lean_flag==1)
				{
					setMaxSpeed(8000);
					setMaxPos(8000);
					// 转换到逆运动学的角度
					transXYToAngle_SUM();
					// 控制腿部运动
					setMotionOutAngle();
					runMotor();
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
						setMaxSpeed(8000);
						setMaxPos(8000);
						// 转换到逆运动学的角度
						transXYToAngle_SUM();
						// 控制腿部运动
						setMotionOutAngle();
						runMotor();
					}
				}
				break;
		}
		
		default:
			break;
	}

}	
