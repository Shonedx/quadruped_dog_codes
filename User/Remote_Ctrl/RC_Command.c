/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "stm32f4xx.h"
#include "Allheaderfile.h"
#include "math.h"

Ctrl_State ctrl_state;
int if_idle=0; //1为原地踏步（idle) 0为停止
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


void Ctrl_Cmd(void) //切换机器狗运动状态的控制器 
{
		//前进
	if (left_y >= 230 && abs(left_x) <= 330 && abs(right_y) <= 330 && abs(right_x) <= 330  ) 
	{	
		currentstate=Forward;
	}
	// 原地左平移
	else if (right_x <= -100 && abs(right_y) <= 230  && abs(left_y) <= 330 && abs(left_x) <= 330 )
	{
		
		currentstate=Translate_Left;
	}
	// 原地右平移
	else if (right_x >= 100  && abs(right_y) <= 230 && abs(left_y) <= 330 && abs(left_x) <= 330) 
	{
		
		currentstate=Translate_Right;
	}
	// 原地左转
	else if (left_x <= -100 && abs(left_y) <= 230  && abs(right_y) <= 330 && abs(right_x) <= 330 ) 
	{
		
		currentstate=Turn_Left;
	}
	// 原地右转
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

   
	



