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
uint8_t check_angle_trigger=0;
uint16_t formal_datas[FORMAL_DATAS_LENGTH]={0};
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
void trans_rx_buffer_to_formal_datas(void) //ת�����յ�����Ϊ��׼��ʽ
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
	rc_left_x=formal_datas[3];	//�е� 2075
	rc_left_y=formal_datas[4];	//�е� 2030
	rc_right_x=formal_datas[1];	//�е� 2050
	rc_right_y=formal_datas[2];	//�е� 2020
	check_angle_trigger=0x01&rx_buffer[11];
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
		if (rc_right_y < rc_right_y_md-100) 
		{
			current_motion_state=MS_TRANSLATE_LEFT;
		}
		// 
		else if (rc_right_y > rc_right_y_md+100) 
		{
			current_motion_state=MS_TRANSLATE_RIGHT;
		}
	}
	

}

void RC_StepLengthCtrl(GaitParams *gaitparams)
{
	if(if_in_normal_range(rc_right_x,rc_right_x_md-100,rc_right_x_md+100)&&if_in_normal_range(rc_right_y,rc_right_y_md-100,rc_right_y_md+100)) //while right rocker don't move 
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