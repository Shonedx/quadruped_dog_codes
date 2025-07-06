#ifndef GaitParams_H
#define GaitParams_H
#include "stdint.h"
typedef enum MotionState
{
	MS_NORMAL,
	MS_TRANSLATE_LEFT,
	MS_TRANSLATE_RIGHT,
	MS_STOP,
}MotionState_t; //����������״̬

//typedef enum 
//{
//	Idle=0,
//	Normal,
//	Translate_Left,
//	Translate_Right,
//	Jump,
//	Stop,
//}State; //����������״̬

typedef struct
{
    double Up_Amp; //Amplitude ��������
    double Down_Amp; //�½��߶�
    double stanceheight; //վ���߶�
    double steplength;  //���Ȳ���
    double freq; //Ƶ��
    double swingpercent; //�ڶ���ռ��
    double gaitoffset;  //���ȵ���λ
    int i; //�����
    double x_offset; //x��ʼֵ
} GaitParams;

extern GaitParams  gait_params[][4];
extern MotionState_t current_motion_state;

void Set_StandHeight(GaitParams *gaitparams,uint8_t height);
void Set_StepLength(GaitParams *gait_params);
#endif


