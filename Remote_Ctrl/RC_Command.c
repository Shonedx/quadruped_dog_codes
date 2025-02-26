

#include "stm32f4xx.h"
#include "Allheaderfile.h"
#include "math.h"

Ctrl_State ctrl_state;
int if_idle=0; //����1ʱ���ʾidle��0Ϊstop״̬

/*		LPS											RPS			*/
/*	�� LPS=2 		 						��RPS=1 // ����		*/
/*�ϵ�ǰң��Ҫ��������*/	
/*	�� LPS=3		 						��RPS=3 //����������	*/
/*ֱ��״̬*/
/*	�� LPS=1		 						��RPS=2 //����������	*/
/*�˶�״̬*/


void Remote_Cmd(void)
{
	if(LPS==2&&RPS==1)
	{
		ctrl_state=Initial_Ctrl;
	}
	else if(LPS==3&&RPS==1)
	{
		ctrl_state=Start_Ctrl;
	}
	else if(LPS==1&&RPS==1)
	{
		ctrl_state=Main_Ctrl;
	}
	else if(LPS==1&&RPS==3)
	{
		ctrl_state=Stop_Ctrl;
	}
	else if(LPS==3&&RPS==3)
	{
		ctrl_state=Jump_Ctrl;
	}
//	else if(LPS==2&&RPS==3)
//	{
//		ctrl_state=JUMP_2;
//	}
}


void Ctrl_Cmd(void) //���������
{
	//ǰ��
	if (left_y >= 230 && abs(left_x) <= 330 && abs(right_y) <= 330 && abs(right_x) <= 330  ) 
	{	
		currentstate=Forward;
	}
	// ��ƽ��
	else if (right_x <= -100 && abs(right_y) <= 230  && abs(left_y) <= 330 && abs(left_x) <= 330 )
	{
		
		currentstate=Translate_Left;
	}
	// ��ƽ��
	else if (right_x >= 100  && abs(right_y) <= 230 && abs(left_y) <= 330 && abs(left_x) <= 330) 
	{
		
		currentstate=Translate_Right;
	}
	// ��ת
	else if (left_x <= -100 && abs(left_y) <= 230  && abs(right_y) <= 330 && abs(right_x) <= 330 ) 
	{
		
		currentstate=Turn_Left;
	}
	// ��ת
	else if (left_x >= 100  && abs(left_y) <= 230  && abs(right_y) <= 330 && abs(right_x) <= 330) 
	{
		
		currentstate=Turn_Right;
	}
	// ����
	else if (left_y <= -230 && abs(left_x) <= 330 && abs(right_y) <= 330 && abs(right_x) <= 330 ) 
	{	
	
		currentstate=Back;
	}
	else if (right_y >= 230 && abs(right_x) <= 330  && abs(left_y) <= 330 && abs(left_x) <= 330 ) 
	{	
		currentstate=Jump;
	}

// ���� 
	else if 
		(
			abs(left_y) <= 330 
			&& abs(left_x) <= 330
			&&abs(right_x) <= 330
			&&abs(right_y) <= 330 
		) 
	{
		if(if_idle==1)
		currentstate=Idle;
		else if(if_idle==0)
		currentstate=Stop;
		TIM_Cmd(TIM4, ENABLE);
	}
	
}

   
	



