#include "string.h"
#include "stdint.h"
#include "NRF24L01.h"
#include "RC.h"
#include "gaitparams.h"
#include "ges_cal.h"
#include "stdbool.h"
#include <math.h>
#define KEY_COUNTS 10 //key 数量
ConnectState_t connect_state=UNCONNECTED;

TranslateState_t translate_state=TRANS_DISABLE;
IdleState_t idle_state=STOP;
CtrlState_t ctrl_state=CS_NONE;
JumpState_t jump_state=IDLE;
uint8_t imu_state=0;
uint8_t setted_height=StandHeight;
uint8_t pre_angle=0;
uint8_t check_angle_trigger=0;
uint8_t key_state_buffer[KEY_COUNTS]={0};
//0-4096
uint16_t rc_left_x=0;	//�е� 2075
uint16_t rc_left_y=0;	//�е� 2030
uint16_t rc_right_x=0;	//�е� 2050
uint16_t rc_right_y=0;	//�е� 2020
//�е�ֵ middle С���е㲿��Ϊ�� or ��
uint16_t rc_left_x_md=2075;	
uint16_t rc_left_y_md=2030;	
uint16_t rc_right_x_md=2050;	
uint16_t rc_right_y_md=2020;	
//
extern uint8_t rx_buffer[NRF_PAYLOAD_LENGTH]; //��������
extern MotionState_t current_motion_state;
extern const float step_length;
//static uint16_t constrain(uint16_t value, uint16_t min, uint16_t max) { //�޷�����
//    if (value < min) return min;
//    if (value > max) return max;
//    return value;
//}
static float constrain(float value, float min, float max) { //�޷�����
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
void normalizeDatas(void) //把数据规范化
{
	jump_state=(rx_buffer[0]>>6)&0x03;
	pre_angle=rx_buffer[10];
	setted_height=rx_buffer[1];
	ctrl_state=rx_buffer[0]&0x0f;
	translate_state=(rx_buffer[0]>>4)&0x01;
	idle_state=(rx_buffer[0]>>5)&0x01;
	rc_left_x=(uint16_t)rx_buffer[7]<<8|((uint16_t)rx_buffer[6]);	//leftx 2075
	rc_left_y=(uint16_t)rx_buffer[9]<<8|((uint16_t)rx_buffer[8]);	//lefty 2030
	rc_right_x=(uint16_t)rx_buffer[3]<<8|((uint16_t)rx_buffer[2]);	//rightx 2050
	rc_right_y=(uint16_t)rx_buffer[5]<<8|((uint16_t)rx_buffer[4]);	//righty 2020
	check_angle_trigger=0x01&rx_buffer[11];
	imu_state=(uint8_t)(0x01&rx_buffer[11]>>1);
	//keys
	key_state_buffer[KY_Enter]=0x01&rx_buffer[11]>>2;
	key_state_buffer[KY_Back]=0x01&rx_buffer[11]>>3;

	key_state_buffer[KY_UP]=0x01&rx_buffer[12];
	key_state_buffer[KY_DOWN]=0x01&rx_buffer[12]>>1;
	key_state_buffer[KY_LEFT]=0x01&rx_buffer[12]>>2;
	key_state_buffer[KY_RIGHT]=0x01&rx_buffer[12]>>3;
	key_state_buffer[KY_Y]=0x01&rx_buffer[12]>>4;
	key_state_buffer[KY_A]=0x01&rx_buffer[12]>>5;
	key_state_buffer[KY_X]=0x01&rx_buffer[12]>>6;
	key_state_buffer[KY_B]=0x01&rx_buffer[12]>>7;
}
uint8_t if_in_normal_range(uint16_t value, uint16_t min, uint16_t max) //�ж�ĳ�����Ƿ���һ����Χ
{
	if(value == constrain(value,min,max))
		return 1;
	return 0;
}
void RC_MotionCtrl(void) //�л��������˶�״̬�Ŀ����� 
{

	if(if_in_normal_range(rc_left_x,rc_left_x_md-500,rc_left_x_md+500) 
		&& if_in_normal_range(rc_left_y,rc_left_y_md-500,rc_left_y_md+500)
		&& if_in_normal_range(rc_right_x,rc_right_x_md-500,rc_right_x_md+500) 
		&& if_in_normal_range(rc_right_y,rc_right_y_md-500,rc_right_y_md+500)) //当两边摇杆没变化
	{	
		if(idle_state==NORMAL)
		current_motion_state=MS_NORMAL; //ԭ��̤��
		else if(idle_state==STOP)
		current_motion_state=MS_STOP;   //ֹͣ
	}
	else //正常运行
	{
		current_motion_state=MS_NORMAL; 
	}
	
	if(translate_state==TRANS_ENABLE) //平移
	{
		// 
		if(!key_state_buffer[KY_LEFT]) //按键默认上拉，按下时接地
		{
			current_motion_state=MS_TRANSLATE_LEFT;
		}
		// 
		else if(!key_state_buffer[KY_RIGHT])
		{
			current_motion_state=MS_TRANSLATE_RIGHT;
		}
	}
	

}

void RC_StepLengthCtrl(GaitParams *gaitparams)
{
	if(!if_in_normal_range(rc_right_x,rc_right_x_md-100,rc_right_x_md+100)&&!if_in_normal_range(rc_left_y,rc_left_y_md-100,rc_left_y_md+100))
	{
		float k1=constrain(((float)rc_right_x-(float)rc_right_x_md)/2048.0f,-1,1);  //右摇杆
		float k2=constrain(((float)rc_left_y-(float)rc_left_y_md)/2048.0f,-1,1); //左摇杆
		
		float ds=(fabs(k2)/fabs(k1))*7.0f;
		if(k1>0) //右摇杆左推
		{
			//left legs
			gaitparams[0].steplength=step_length-ds;
			gaitparams[2].steplength=step_length-ds;
			//right legs
			gaitparams[1].steplength=step_length;
			gaitparams[3].steplength=step_length;
		}
		else if(k1<0) //右摇杆右推
		{
			//left legs
			gaitparams[0].steplength=step_length;
			gaitparams[2].steplength=step_length;
			//right legs
			gaitparams[1].steplength=step_length-ds;
			gaitparams[3].steplength=step_length-ds;
		}
		if(k2<0) //当左摇杆向后拨时
		{
			for(int i=0;i<4;i++)
			{
				gait_params[i]->steplength*=-1;
			}
		}
		
	}
	else if(if_in_normal_range(rc_right_x,rc_right_x_md-100,rc_right_x_md+100)&&if_in_normal_range(rc_right_y,rc_right_y_md-100,rc_right_y_md+100)) //while right rocker don't move 
	{
		float K=constrain(((float)rc_left_y-(float)rc_left_y_md)/2048.0f,-1,1);
		//left legs
		gaitparams[0].steplength=K*step_length;
		gaitparams[2].steplength=K*step_length;
		//right legs
		gaitparams[1].steplength=K*step_length;
		gaitparams[3].steplength=K*step_length;
	} 
	else if(if_in_normal_range(rc_left_x,rc_left_x_md-100,rc_left_x_md+100)&&if_in_normal_range(rc_left_y,rc_left_y_md-100,rc_left_y_md+100)) //while left rocker don't move 
	{
		float K=constrain(((float)rc_right_x-(float)rc_right_x_md)/2048.0f,-1,1);  
		//left legs
		gaitparams[0].steplength=-K*step_length;
		gaitparams[2].steplength=-K*step_length;
		//right legs
		gaitparams[1].steplength=K*step_length;
		gaitparams[3].steplength=K*step_length;
	}
}