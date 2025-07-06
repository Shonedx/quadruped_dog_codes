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
#include "RC.h"
#include "RC_COMMAND.h"
#include "gaitparams.h"


extern int left_x, left_y, right_x, right_y; 


/*		LPS											RPS			*/
/*	上 LPS=2 		 						上RPS=1 // 奔跑		*/
/*上电前遥杆要拨到这里*/	
/*	中 LPS=3		 						中RPS=3 //拨到这是走	*/
/*直立状态*/
/*	下 LPS=1		 						下RPS=2 //拨到这是跳	*/
/*运动状态*/

//void RC_LevelCtrl(void) //遥控器拨杆控制状态
//{
//	
//	if(LPS==2&&RPS==1)
//	{
//		ctrl_state=CS_INITIAL;
//	}
//	else if(LPS==3&&RPS==1)
//	{
//		ctrl_state=CS_START;
//	}
//	else if(LPS==1&&RPS==1)
//	{
//		ctrl_state=CS_MAIN;
//	}
//	else if(LPS==1&&RPS==3)
//	{
//		ctrl_state=CS_STOP;
//	}
//	else if(LPS==3&&RPS==3)
//	{
//		ctrl_state=CS_PRE_JUMP;
//	}
//	else if(LPS==2&&RPS==3)
//	{
//		ctrl_state=CS_EXE_JUMP;
//	}
//	else if(LPS==1&&RPS==2)
//	{
//		ctrl_state=CS_CROUCH;
//	}
//	else if(LPS==3&&RPS==2)
//	{
//		ctrl_state=CS_HIGHER;
//	}
//}


//void RC_MotionCtrl(void) //切换机器狗运动状态的控制器 
//{
//	if(abs(right_x)<=100&&abs(right_y)<=100)
//	{
//		if 
//		(
//			abs(left_y) <= 100 
//			&& abs(left_x) <= 100
//		) 
//		{	
//			if(if_idle==1)
//			current_motion_state=MS_NORMAL; //原地踏步
//			else if(if_idle==0)
//			current_motion_state=MS_STOP;   //停止
//		}
//		else
//		{
//			current_motion_state=MS_NORMAL; 
//		}
//	}
//	else
//	{
//		// 原地左平移
//		if (right_x <= -100)
//		{
//			
//			current_motion_state=MS_TRANSLATE_LEFT;
//		}
//		// 原地右平移
//		else if (right_x >= 100) 
//		{
//			
//			current_motion_state=MS_TRANSLATE_RIGHT;
//		}
//	}
//	if((right_y)<=-100&&abs(right_x)<=100)
//	{
//		current_motion_state=MS_NORMAL;
//	}
//}

   
	



