

#include "stm32f4xx.h"
#include "Allheaderfile.h"
#include "math.h"

Ctrl_State ctrl_state;
int if_idle=0; //等于1时候表示idle，0为stop状态

/*		LPS											RPS			*/
/*	上 LPS=2 		 						上RPS=1 // 奔跑		*/
/*上电前遥杆要拨到这里*/	
/*	中 LPS=3		 						中RPS=3 //拨到这是走	*/
/*直立状态*/
/*	下 LPS=1		 						下RPS=2 //拨到这是跳	*/
/*运动状态*/


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


void Ctrl_Cmd(void) //控制命令函数
{
	//前进
	if (left_y >= 230 && abs(left_x) <= 330 && abs(right_y) <= 330 && abs(right_x) <= 330  ) 
	{	
		currentstate=Forward;
	}
	// 左平移
	else if (right_x <= -100 && abs(right_y) <= 230  && abs(left_y) <= 330 && abs(left_x) <= 330 )
	{
		
		currentstate=Translate_Left;
	}
	// 右平移
	else if (right_x >= 100  && abs(right_y) <= 230 && abs(left_y) <= 330 && abs(left_x) <= 330) 
	{
		
		currentstate=Translate_Right;
	}
	// 左转
	else if (left_x <= -100 && abs(left_y) <= 230  && abs(right_y) <= 330 && abs(right_x) <= 330 ) 
	{
		
		currentstate=Turn_Left;
	}
	// 右转
	else if (left_x >= 100  && abs(left_y) <= 230  && abs(right_y) <= 330 && abs(right_x) <= 330) 
	{
		
		currentstate=Turn_Right;
	}
	// 后退
	else if (left_y <= -230 && abs(left_x) <= 330 && abs(right_y) <= 330 && abs(right_x) <= 330 ) 
	{	
	
		currentstate=Back;
	}
	else if (right_y >= 230 && abs(right_x) <= 330  && abs(left_y) <= 330 && abs(left_x) <= 330 ) 
	{	
		currentstate=Jump;
	}

// 重置 
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

   
	



