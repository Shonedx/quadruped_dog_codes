#ifndef GaitParams_H
#define GaitParams_H
#include "Allheaderfile.h"
typedef enum 
{
	Idle=0,
	Forward, 
	Back,
	Turn_Left,
	Turn_Right,
	Translate_Left,
	Translate_Right,
	Jump,
	Stop,
}State; //����������״̬


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

extern GaitParams  gaitparams[][4];
extern State currentstate;

#endif