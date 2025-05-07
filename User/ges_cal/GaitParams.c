#include "GaitParams.h"
State currentstate;

GaitParams gaitparams[][4] = {
  //	Up_Amp Down_Amp stanceheight steplength freq swingpercent gaitoffset i x_offset
  //	�������� �½��߶� վ���߶� ���Ȳ��� Ƶ�� �ڶ���ռ�� ���ȵ���λ ����� x��ʼֵ
{//ԭ��̤����������״̬ 0
	{ 3.5, 0.2, StandHeight, 0, Freq*1.0, 0.25, 0, 0,X_OFFSET},
	{ 3.5, 0.2, StandHeight, 0, Freq*1.0, 0.25, 0.5, 1,X_OFFSET},
	{ 3.5, 0.2, StandHeight, 0, Freq*1.0, 0.25, 0.5, 2,X_OFFSET},
	{ 3.5, 0.2, StandHeight, 0, Freq*1.0, 0.25, 0, 3,X_OFFSET},
},
  {//ǰ��״̬ 1
	{ 4.0, 0.2, StandHeight, 5, Freq, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq, 0.25, 0, 3,X_OFFSET},
},
  {//����״̬ 2
	{ 4.0, 0.2, StandHeight, -5, Freq, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.2, StandHeight,-5, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.2, StandHeight, -5, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.2, StandHeight, -5, Freq, 0.25, 0, 3,X_OFFSET},
},
{//��ת��״̬ 3
	{ 4.0, 0.2, StandHeight, -5, Freq*1.2, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq*1.2, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.2, StandHeight, -5, Freq*1.2, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq*1.2, 0.25, 0, 3,X_OFFSET},
},
{//��ת��״̬ 4
	{ 4.0, 0.2, StandHeight,5, Freq*1.2, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.2, StandHeight, -5,Freq*1.2, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq*1.2, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.2, StandHeight, -5, Freq*1.2, 0.25, 0, 3,X_OFFSET},
},

{//ֹͣ 5
	{ 0, 0, StandHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 3,X_OFFSET},
},
{//��ƽ��״̬ 6
	{ 2, 0.2, StandHeight+2, 0, Freq, 0.25, 0.5, 0,X_OFFSET},
	{ 2, 0.2, StandHeight-2, 0, Freq, 0.25, 0, 1,X_OFFSET},
	{ 2, 0.2, StandHeight+2, 0, Freq ,0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.2, StandHeight-2, 0, Freq, 0.25, 0, 3,X_OFFSET},
},
{//��ƽ��״̬ 7
	{ 2, 0.2, StandHeight-2, 0, Freq, 0.25, 0.5, 0,X_OFFSET},
	{ 2, 0.2, StandHeight+2, 0, Freq, 0.25, 0, 1,X_OFFSET},
	{ 2, 0.2, StandHeight-2, 0, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.2, StandHeight+2, 0, Freq, 0.25, 0, 3,X_OFFSET},
},

////crouch

{//ǰ��״̬ 1+7
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0, 0,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0, 3,X_OFFSET},
},
  {//����״̬ 2+7
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0, 0,X_OFFSET},
	{ 2, 0.1, CrouchHeight,-4, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0, 3,X_OFFSET},
},
{//��ת��״̬ 3+7
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0, 0,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0, 3,X_OFFSET},
},
{//��ת��״̬ 4+7
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0, 0,X_OFFSET},
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0, 3,X_OFFSET},
},
{//ֹͣ״̬ 5+7
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 3,X_OFFSET},
},

//Higher
{//ǰ��״̬ 1+12
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight,8, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0, 3,X_OFFSET},
},
  {//����״̬ 2+12
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight,-8, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
	{4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0, 3,X_OFFSET},
},
{//��ת��״̬ 3+12
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0, 3,X_OFFSET},
},
{//��ת��״̬ 4+12
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0, 3,X_OFFSET},
},
{//ֹͣ״̬ 5+12
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 3,X_OFFSET},
},
 };