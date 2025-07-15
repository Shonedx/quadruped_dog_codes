#include "string.h"
#include "stdint.h"
#include "rc_nrf24l01.h"
#include "RC.h"
#include "GaitParams.h"
#include "motion_ctrl.h"
#include "rc_spi.h"
TranslateState_t translate_state=TRANS_DISABLE;
IdleState_t idle_state=STOP;
CtrlState_t ctrl_state=CS_NONE;
JumpState_t jump_state=IDLE;

u8 pre_height=StandHeight; // 预设高度
u8 pre_angle=0; // 预设角度

//0-4096
u16 rc_left_x=0;	//中点 2075
u16 rc_left_y=0;	//中点 2030
u16 rc_right_x=0;	//中点 2050
u16 rc_right_y=0;	//中点 2020
//中点值 middle 小于中点部分为左 or 上
u16 rc_left_x_md=2075;	
u16 rc_left_y_md=2030;	
u16 rc_right_x_md=2050;	
u16 rc_right_y_md=2020;	
//
extern u8 rx_buffer[NRF_PAYLOAD_LENGTH]; //接收数组
extern MotionState_t current_motion_state;
static float constrain(float value, float min, float max) { //限幅函数
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void formalDataFromRxBuffer(void) //转换接收的数据为标准格式
{
	//转换成要用的数据
	jump_state=(rx_buffer[0]>>6)&0x03;
    ctrl_state=rx_buffer[0]&0x0f;
	translate_state=(rx_buffer[0]>>4)&0x01;
	idle_state=(rx_buffer[0]>>5)&0x01;

	pre_angle=rx_buffer[10];
	pre_height=rx_buffer[1];

	rc_left_x=(u16)rx_buffer[7]<<8|((u16)rx_buffer[6]);	//中点 2075
	rc_left_y=(u16)rx_buffer[9]<<8|((u16)rx_buffer[8]);	//中点 2030
	rc_right_x=(u16)rx_buffer[3]<<8|((u16)rx_buffer[2]);	//中点 2050
	rc_right_y=(u16)rx_buffer[5]<<8|((u16)rx_buffer[4]);	//中点 2020
}
u8 judgeNumInRange(u16 value, u16 min, u16 max) //判断某个数是否在一定范围
{
	if(value == constrain(value,min,max))
		return 1;
	return 0;
}

void rcMotionCtrl(void) //切换机器狗运动状态的控制器 
{
	if(judgeNumInRange(rc_left_x,rc_left_x_md-500,rc_left_x_md+500) && judgeNumInRange(rc_left_y,rc_left_y_md-500,rc_left_y_md+500))
	{	
		if(idle_state==NORMAL)
		current_motion_state=MS_NORMAL; //原地踏步
		else if(idle_state==STOP)
		current_motion_state=MS_STOP;   //停止
	}
	else if (judgeNumInRange(rc_right_x,rc_right_x_md-500,rc_right_x_md+500) && judgeNumInRange(rc_right_y,rc_right_y_md-500,rc_right_y_md+500))
	{
		current_motion_state=MS_NORMAL; 
	}
	if(translate_state==TRANS_ENABLE)
	{
		// 原地左平移
		if (rc_right_x < rc_right_x_md-100) //100是偏移值
		{
			current_motion_state=MS_TRANSLATE_LEFT;
		}
		// 原地右平移
		else if (rc_right_x > rc_right_x_md+100) 
		{
			current_motion_state=MS_TRANSLATE_RIGHT;
		}
	}

}
 
void rcSetStepLength(GaitParams_t *gait_params) //步长控制
{
	if(judgeNumInRange(rc_left_x,rc_left_x_md-100,rc_left_x_md+100)) //当左摇杆只往纵轴拨时 前进 or 后退
	{
		float K=constrain(-((float)rc_left_y-(float)rc_left_y_md)/2048.0f,-1,1);
		//left legs
		gait_params[0].step_length=K*StepLength;
		gait_params[2].step_length=K*StepLength;
		//right legs
		gait_params[1].step_length=K*StepLength;
		gait_params[3].step_length=K*StepLength;
	}
	else//转弯
	{
		float K=constrain(((float)rc_left_x-(float)rc_left_x_md)/2048.0f,-1,1); 
		//left legs
		gait_params[0].step_length=K*StepLength;
		gait_params[2].step_length=K*StepLength;
		//right legs
		gait_params[1].step_length=-K*StepLength;
		gait_params[3].step_length=-K*StepLength;
	}
	if(judgeNumInRange(rc_left_x,rc_left_x_md-100,rc_left_x_md+100)&&judgeNumInRange(rc_left_y,rc_left_y_md-100,rc_left_y_md+100)) //当左摇杆没咋动时
	{
		//left legs
		gait_params[0].step_length*=0;
		gait_params[2].step_length*=0;
		//right legs
		gait_params[1].step_length*=0;
		gait_params[3].step_length*=0;
	}
}