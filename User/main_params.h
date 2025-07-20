#ifndef __MAIN_PARAMS_H
#define __MAIN_PARAMS_H
#include "Allheaderfile.h"

//ע:���ļ�����ͳһ��main�������ı����ŵ�ͷ�ļ������Ը������ļ����ñ�������������Ͳ鿴

extern uint8_t rx_buffer[NRF_PAYLOAD_LENGTH];

extern 	int start;
extern int feed;
extern  int reset;	

extern	int left_x,left_y,right_x,right_y; //RemoteControl��������
extern 	int left_push_stick, right_push_stick; //���ϣ������Ƹ�

#endif
