

#include "Allheaderfile.h"
#include "ticks.h"
#include "timer.h"



//Ft=168Mhz/4*时钟分频

void TIM4_Init(u16 arr,u16 psc) 
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM4时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = 65535; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	TIM_Cmd(TIM4,DISABLE); //使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02; //子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
void TIM5_Init(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	
	
	TIM_Cmd(TIM5,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM4_IRQHandler() //控制类逻辑
{  
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) !=RESET)
	{
		if(start==1)
		Gait(); //执行步态逻辑
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}


void TIM5_IRQHandler(void) //状态机
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET)
	{		

	switch(ctrl_state)
	{
			case Initial_Ctrl://初始状态 
			M3508_ALL_ZERO_SET();
			start=0;
			if_idle=0;
            break;
			case Start_Ctrl://初始化机器狗
			start=1;
			if_idle=0;
			Ctrl_Cmd();
			break;
			
			case Main_Ctrl://主控制
			if_idle=1;
			start=1;
			Ctrl_Cmd();
			break;
			
			case Stop_Ctrl:// 停止
			if_idle=0;
			start=1;
			Ctrl_Cmd();
			break;
			
			case Jump_Ctrl:// 跳跃
			if_idle=0;
			start=1;
			Jump_OK=1;
			Ctrl_Cmd();
			break;
	}
		if(start==1)
		{
			Motor_Auto_Run();
		}
		feed_dog();
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //清除中断标志位
}


void feed_dog(void)
{
	if(feed)
	{
		IWDG_Feed();	
		LED1 =0;
	}
	else
	{
		LED1 = 1;
	}

}


