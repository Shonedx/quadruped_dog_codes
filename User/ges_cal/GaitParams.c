#include "GaitParams.h"
#include "ges_cal.h"
#include "Allheaderfile.h"
#include "rc.h"

MotionState_t current_motion_state=MS_NORMAL;
const float step_length=16; //�������߲���
const float higher_step_length=10; //�������߲���
const float crouch_step_length=4; //�¶����߲���
const float trun_step_length=6; //ת�䲽��
const float up_amp=8;
const float higher_up_amp=5;
const float crouch_up_amp=2.5;
GaitParams gait_params[][4] = {
  //	Up_Amp Down_Amp stanceheight steplength freq swingpercent gaitoffset i x_offset
	//	上摆幅度 下摆幅度

 {//Normal 0
	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0, 0,X_OFFSET},
	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0.5, 1,X_OFFSET},
	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0.5, 2,X_OFFSET},
	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0, 3,X_OFFSET},
},
{//停止	1
	{ 0, 0, StandHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 3,X_OFFSET},
},
{//左平移 2
	{ up_amp, 0.2, StandHeight-2, 0, Freq*1.5, 0.5, 0.5, 0,X_OFFSET},
	{ up_amp, 0.2, StandHeight+2, 0, Freq*1.5, 0.5, 0, 1,X_OFFSET},
	{ up_amp, 0.2, StandHeight-2, 0, Freq*1.5 ,0.5, 0.5, 2,X_OFFSET},
	{ up_amp, 0.2, StandHeight+2, 0, Freq*1.5, 0.5, 0, 3,X_OFFSET},
},
{//右平移 3
	{ up_amp, 0.2, StandHeight+2, 0, Freq*1.5, 0.5, 0.5, 0,X_OFFSET},
	{ up_amp, 0.2, StandHeight-2, 0, Freq*1.5, 0.5, 0, 1,X_OFFSET},
	{ up_amp, 0.2, StandHeight+2, 0, Freq*1.5, 0.5, 0.5, 2,X_OFFSET},
	{ up_amp, 0.2, StandHeight-2, 0, Freq*1.5, 0.5, 0, 3,X_OFFSET},
},

//crouch

{// Normal 0+4
	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0, 0,X_OFFSET},
	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0.5, 1,X_OFFSET},
	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0.5, 2,X_OFFSET},
	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0, 3,X_OFFSET},
},
{//停止 1+4
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 3,X_OFFSET},
},

//Higher

{//Normal 0+6
	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0, 0,X_OFFSET},
	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
	{ higher_up_amp, 0.1, HeigherHeight,higher_step_length, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0, 3,X_OFFSET},
},
{//停止 1+6
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 3,X_OFFSET},
},
 };
 void Set_StandHeight(GaitParams *gaitparams,uint8_t height)
 {
	//left legs
	gaitparams[0].stanceheight=height;
	gaitparams[2].stanceheight=height;
	//right legs
	gaitparams[1].stanceheight=height;
	gaitparams[3].stanceheight=height;
 }
 
extern uint16_t rc_left_x,rc_left_y,rc_right_x,rc_right_y;
//中点值 middle 小于中点部分为左 or 上
extern uint16_t rc_left_x_md,rc_left_y_md,rc_right_x_md,rc_right_y_md;	
 
static float constrain(float value, float min, float max) { //限幅函数
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void Set_StepLength(GaitParams *gaitparams)
{
	if(if_in_normal_range(rc_left_x,rc_left_x_md-100,rc_left_x_md+100)) //当左摇杆只往纵轴拨时 前进 or 后退
	{
		float K=constrain(-((float)rc_left_y-(float)rc_left_y_md)/2048.0f,-1,1);
		//left legs
		gaitparams[0].steplength=K*step_length;
		gaitparams[2].steplength=K*step_length;
		//right legs
		gaitparams[1].steplength=K*step_length;
		gaitparams[3].steplength=K*step_length;
	}
	else//转弯
	{
		float K=constrain(
			((float)rc_left_x-(float)rc_left_x_md)/2048.0f,-1,1); 
		//left legs
		gaitparams[0].steplength=K*step_length;
		gaitparams[2].steplength=K*step_length;
		//right legs
		gaitparams[1].steplength=-K*step_length;
		gaitparams[3].steplength=-K*step_length;
	}
	if(if_in_normal_range(rc_left_x,rc_left_x_md-100,rc_left_x_md+100)&&if_in_normal_range(rc_left_y,rc_left_y_md-100,rc_left_y_md+100)) //当左摇杆没咋动时
	{
		//left legs
		gaitparams[0].steplength*=0;
		gaitparams[2].steplength*=0;
		//right legs
		gaitparams[1].steplength*=0;
		gaitparams[3].steplength*=0;
	}
 }