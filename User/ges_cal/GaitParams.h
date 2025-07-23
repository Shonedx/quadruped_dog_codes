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
    float Up_Amp; //Amplitude ��������
    float Down_Amp; //�½��߶�
    float stanceheight; //վ���߶�
    float steplength;  //���Ȳ���
    float freq; //Ƶ��
    float swingpercent; //�ڶ���ռ��
    float gaitoffset;  //���ȵ���λ
    int i; //�����
    float x_offset; //x��ʼֵ
} GaitParams;

extern GaitParams  gait_params[][4];
extern MotionState_t current_motion_state;

void Set_StandHeight(GaitParams *gaitparams,uint8_t tar_height);
void Set_StepLength(GaitParams *gaitparams);
void Set_UpAmpLength(GaitParams *gaitparams);

#endif


