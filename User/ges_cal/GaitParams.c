#include "GaitParams.h"
#include "ges_cal.h"
#include "Allheaderfile.h"
#include "rc.h"

MotionState_t current_motion_state=MS_NORMAL;
const float step_length=18; //正常站立高度时的步长
const float up_amp=7.5;  //腿的上摆
//const float higher_step_length=10; //�������߲���
//const float crouch_step_length=4; //�¶����߲���


//const float higher_up_amp=5;
//const float crouch_up_amp=2.5;
GaitParams gait_params[][4] = {
  //	Up_Amp Down_Amp stanceheight steplength freq swingpercent gaitoffset i x_offset
	//	上摆幅度 下摆幅度

	{//Normal 0
		{ up_amp, 0.2, StandHeight, step_length, Freq, 0.45, 0, 0,X_OFFSET},
		{ up_amp, 0.2, StandHeight, step_length, Freq, 0.45, 0.5, 1,X_OFFSET},
		{ up_amp, 0.2, StandHeight, step_length, Freq, 0.45, 0.5, 2,X_OFFSET},
		{ up_amp, 0.2, StandHeight, step_length, Freq, 0.45, 0, 3,X_OFFSET},
	},
	{//停止	1
		{ 0, 0, StandHeight, 0, 0, 0, 0, 0,X_OFFSET},
		{ 0, 0, StandHeight, 0, 0, 0, 0, 1,X_OFFSET},
		{ 0, 0, StandHeight, 0, 0, 0, 0, 2,X_OFFSET},
		{ 0, 0, StandHeight, 0, 0, 0, 0, 3,X_OFFSET},
	},
	{//左平移 2
		{ up_amp, 0.2, StandHeight, 0, Freq*1.5, 0.5, 0.5, 0,X_OFFSET},
		{ up_amp, 0.2, StandHeight+4, 0, Freq*1.5, 0.5, 0, 1,X_OFFSET},
		{ up_amp, 0.2, StandHeight, 0, Freq*1.5, 0.5, 0.5, 2,X_OFFSET},
		{ up_amp, 0.2, StandHeight+4, 0, Freq*1.5, 0.5, 0, 3,X_OFFSET},
	},
	{//右平移 3
		{ up_amp, 0.2, StandHeight+4.0, 0, Freq*1.5, 0.5, 0.5, 0,X_OFFSET},
		{ up_amp, 0.2, StandHeight, 0, Freq*1.5, 0.5, 0, 1,X_OFFSET},
		{ up_amp, 0.2, StandHeight+4.0, 0, Freq*1.5, 0.5, 0.5, 2,X_OFFSET},
		{ up_amp, 0.2, StandHeight, 0, Freq*1.5, 0.5, 0, 3,X_OFFSET},
	},

////crouch

//{// Normal 0+4
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0, 0,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0.5, 1,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0.5, 2,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0, 3,X_OFFSET},
//},
//{//停止 1+4
//	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 0,X_OFFSET},
//	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 1,X_OFFSET},
//	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 2,X_OFFSET},
//	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 3,X_OFFSET},
//},

////Higher

//{//Normal 0+6
//	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0, 0,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight,higher_step_length, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0, 3,X_OFFSET},
//},
//{//停止 1+6
//	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 0,X_OFFSET},
//	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 1,X_OFFSET},
//	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 2,X_OFFSET},
//	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 3,X_OFFSET},
//},
 };
static float constrain(float value, float min, float max) { //限幅函数
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
void Set_StandHeight(GaitParams *gaitparams,uint8_t tar_height)
{
	//left legs
	gaitparams[0].stanceheight=tar_height;
	gaitparams[2].stanceheight=tar_height;
	//right legs
	gaitparams[1].stanceheight=tar_height;
	gaitparams[3].stanceheight=tar_height;
}
//步长随着高度变化而变化
void Set_StepLength(GaitParams *gaitparams)
{
	//left legs
	gaitparams[0].steplength=gaitparams[0].stanceheight*0.8;
	gaitparams[2].steplength=gaitparams[2].stanceheight*0.8;
	//right legs
	gaitparams[1].steplength=gaitparams[1].stanceheight*0.8;
	gaitparams[3].steplength=gaitparams[3].stanceheight*0.8;

	
}
//上摆幅度随着高度变化而变化
void Set_UpAmpLength(GaitParams *gaitparams)
{
	//left legs
	gaitparams[0].Up_Amp=0.25*gaitparams[0].stanceheight;
	gaitparams[2].Up_Amp=0.25*gaitparams[2].stanceheight;
	//right legs
	gaitparams[1].Up_Amp=0.25*gaitparams[1].stanceheight;
	gaitparams[3].Up_Amp=0.25*gaitparams[3].stanceheight;
}
