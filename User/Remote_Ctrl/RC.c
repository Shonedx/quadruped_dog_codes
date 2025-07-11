#include "string.h"
#include "stdint.h"
#include "NRF24L01.h"
#include "RC.h"
#include "gaitparams.h"
#include "ges_cal.h"
ConnectState_t connect_state=UNCONNECTED;

TranslateState_t translate_state=TRANS_DISABLE;
IdleState_t idle_state=STOP;
CtrlState_t ctrl_state=CS_NONE;
JumpState_t jump_state=IDLE;
uint8_t setted_height=StandHeight;
uint8_t pre_angle=0;

uint16_t formal_datas[FORMAL_DATAS_LENGTH]={0};
//0-4096
uint16_t rc_left_x=0;	//中点 2075
uint16_t rc_left_y=0;	//中点 2030
uint16_t rc_right_x=0;	//中点 2050
uint16_t rc_right_y=0;	//中点 2020
//中点值 middle 小于中点部分为左 or 上
uint16_t rc_left_x_md=2075;	
uint16_t rc_left_y_md=2030;	
uint16_t rc_right_x_md=2050;	
uint16_t rc_right_y_md=2020;	
//
extern uint8_t rx_buffer[NRF_PAYLOAD_LENGTH]; //接收数组
extern MotionState_t current_motion_state;
extern const float step_length;
//static uint16_t constrain(uint16_t value, uint16_t min, uint16_t max) { //限幅函数
//    if (value < min) return min;
//    if (value > max) return max;
//    return value;
//}
static float constrain(float value, float min, float max) { //限幅函数
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
void trans_rx_buffer_to_formal_datas(void) //转换接收的数据为标准格式
{
	formal_datas[0]=rx_buffer[0]&0x0f;
	formal_datas[1]=(uint16_t)rx_buffer[3]<<8|((uint16_t)rx_buffer[2]); //rightx
	formal_datas[2]=(uint16_t)rx_buffer[5]<<8|((uint16_t)rx_buffer[4]); 	//righty
	formal_datas[3]=(uint16_t)rx_buffer[7]<<8|((uint16_t)rx_buffer[6]); //leftx
	formal_datas[4]=(uint16_t)rx_buffer[9]<<8|((uint16_t)rx_buffer[8]); 	//lefty
	formal_datas[5]=(rx_buffer[0]>>4)&0x01; //translate state
	formal_datas[6]=(rx_buffer[0]>>5)&0x01; //idle state
	formal_datas[7]=rx_buffer[1]; //setted_height 
	formal_datas[8]=rx_buffer[10]; //pre_angle
	formal_datas[9]=(rx_buffer[0]>>6)&0x03; //jump_state
	//
	jump_state=formal_datas[9];
	pre_angle=(formal_datas[8]);
	setted_height=(formal_datas[7]);
	ctrl_state=formal_datas[0];
	translate_state=formal_datas[5];
	idle_state=formal_datas[6];
	rc_left_x=formal_datas[3];	//中点 2075
	rc_left_y=formal_datas[4];	//中点 2030
	rc_right_x=formal_datas[1];	//中点 2050
	rc_right_y=formal_datas[2];	//中点 2020
}
uint8_t if_in_normal_range(uint16_t value, uint16_t min, uint16_t max) //判断某个数是否在一定范围
{
	if(value == constrain(value,min,max))
		return 1;
	return 0;
}
void RC_MotionCtrl(void) //切换机器狗运动状态的控制器 
{

	if(if_in_normal_range(rc_left_x,rc_left_x_md-500,rc_left_x_md+500) && if_in_normal_range(rc_left_y,rc_left_y_md-500,rc_left_y_md+500))
	{	
		if(idle_state==NORMAL)
		current_motion_state=MS_NORMAL; //原地踏步
		else if(idle_state==STOP)
		current_motion_state=MS_STOP;   //停止
	}
	else if (if_in_normal_range(rc_right_x,rc_right_x_md-500,rc_right_x_md+500) && if_in_normal_range(rc_right_y,rc_right_y_md-500,rc_right_y_md+500))
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
extern uint16_t rc_left_x,rc_left_y,rc_right_x,rc_right_y;
//中点值 middle 小于中点部分为左 or 上
extern uint16_t rc_left_x_md,rc_left_y_md,rc_right_x_md,rc_right_y_md;	
 
void RC_StepLengthCtrl(GaitParams *gaitparams)
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
		float K=constrain(((float)rc_left_x-(float)rc_left_x_md)/2048.0f,-1,1); 
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