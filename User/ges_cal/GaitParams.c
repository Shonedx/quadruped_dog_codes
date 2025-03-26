#include "GaitParams.h"
State currentstate;

GaitParams gaitparams[][4] = {
  //	Up_Amp Down_Amp stanceheight steplength freq swingpercent gaitoffset i x_offset
  //	�������� �½��߶� վ���߶� ���Ȳ��� Ƶ�� �ڶ���ռ�� ���ȵ���λ ����� x��ʼֵ
{//ԭ��̤����������״̬ 0
	{ 4, 0.2, StandHeight, 0, Freq, 0.25, 0, 0,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq, 0.25, 0, 3,X_OFFSET},
},
  {//ǰ��״̬ 1
	{ 5, 0.2, StandHeight, 10, Freq, 0.25, 0, 0,X_OFFSET},
	{ 5, 0.2, StandHeight, 10, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 5, 0.2, StandHeight, 10, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 5, 0.2, StandHeight, 10, Freq, 0.25, 0, 3,X_OFFSET},
},
  {//����״̬ 2
	{ 5, 0.2, StandHeight, -10, Freq, 0.25, 0, 0,X_OFFSET},
	{ 5, 0.2, StandHeight,-10, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 5, 0.2, StandHeight, -10, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 5, 0.2, StandHeight, -10, Freq, 0.25, 0, 3,X_OFFSET},
},
{//��ת��״̬ 3
	{ 4, 0.2, StandHeight, -8, Freq, 0.25, 0, 0,X_OFFSET},
	{ 4, 0.2, StandHeight, 8, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 4, 0.2, StandHeight, -8, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 4, 0.2, StandHeight, 8, Freq, 0.25, 0, 3,X_OFFSET},
},
{//��ת��״̬ 4
	{ 4, 0.2, StandHeight, 8, Freq, 0.25, 0, 0,X_OFFSET},
	{ 4, 0.2, StandHeight, -8,Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 4, 0.2, StandHeight, 8, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 4, 0.2, StandHeight, -8, Freq, 0.25, 0, 3,X_OFFSET},
},

{//ֹͣ 5
	{ 0, 0, StandHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 3,X_OFFSET},
},
{//��ƽ��״̬ 6
	{ 4, 0.2, StandHeight, 0, Freq+0.5, 0.25, 0, 0,X_OFFSET},
	{ 4, 0.2, StandHeight-3.5, 0, Freq+0.5, 0.25, 0.5, 1,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq+0.5, 0.25, 0.5, 2,X_OFFSET},
	{ 4, 0.2, StandHeight-3.5, 0, Freq+0.5, 0.25, 0, 3,X_OFFSET},
},
{//��ƽ��״̬ 7
	{ 4, 0.2, StandHeight-3.5, 0, Freq+0.5, 0.25, 0, 0,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq+0.5, 0.25, 0.5, 1,X_OFFSET},
	{ 4, 0.2, StandHeight-3.5, 0, Freq+0.5, 0.25, 0.5, 2,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq+0.5, 0.25, 0, 3,X_OFFSET},
},

////Climbing
//{//ԭ��̤����������״̬ 0+6
//	{ 3, 0.1, StandHeight, 0, 3, 0.25, 0, 0,X_OFFSET},
//	{ 3, 0.1, StandHeight, 0, 3, 0.25, 0.5, 1,X_OFFSET},
//	{ 3, 0.1, StandHeight, 0, 3, 0.25, 0.5, 2,X_OFFSET},
//	{ 3, 0.1, StandHeight, 0, 3, 0.25, 0, 3,X_OFFSET},
//},
//{//ǰ��״̬ 1+6
//	{ 3, 0.1, StandHeight, 3, 3, 0.25, 0, 0,X_OFFSET},
//	{ 3, 0.1, StandHeight, 3, 3, 0.25, 0.5, 1,X_OFFSET},
//	{ 3, 0.1, StandHeight, 3, 3, 0.25, 0.5, 2,X_OFFSET},
//	{ 3, 0.1, StandHeight, 3, 3, 0.25, 0, 3,X_OFFSET},
//},
//  {//����״̬ 2+6
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0, 0,X_OFFSET},
//	{ 3, 0.1, StandHeight,-4, 3, 0.25, 0.5, 1,X_OFFSET},
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0.5, 2,X_OFFSET},
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0, 3,X_OFFSET},
//},
//{//��ת��״̬ 3+6
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0, 0,X_OFFSET},
//	{ 3, 0.1, StandHeight, 4, 3, 0.25, 0.5, 1,X_OFFSET},
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0.5, 2,X_OFFSET},
//	{ 3, 0.1, StandHeight, 4, 3, 0.25, 0, 3,X_OFFSET},
//},
//{//��ת��״̬ 4+6
//	{ 3, 0.1, StandHeight, 4, 3, 0.25, 0, 0,X_OFFSET},
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0.5, 1,X_OFFSET},
//	{ 3, 0.1, StandHeight, 4, 3, 0.25, 0.5, 2,X_OFFSET},
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0, 3,X_OFFSET},
//},
 };