

#include "Allheaderfile.h"
#include "ticks.h"
#include "timer.h"



//Ft=168Mhz/4*ʱ�ӷ�Ƶ

void TIM4_Init(u16 arr,u16 psc) 
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///ʹ��TIM4ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = 65535; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //����ʱ��4�����ж�
	TIM_Cmd(TIM4,DISABLE); //ʹ�ܶ�ʱ��4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //��ʱ��4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02; //�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
void TIM5_Init(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	
	
	TIM_Cmd(TIM5,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM4_IRQHandler() //�������߼�
{  
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) !=RESET)
	{
		if(start==1)
		Gait(); //ִ�в�̬�߼�
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}


void TIM5_IRQHandler(void) //״̬��
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET)
	{		

	switch(ctrl_state)
	{
			case Initial_Ctrl://��ʼ״̬ 
			M3508_ALL_ZERO_SET();
			start=0;
			if_idle=0;
            break;
			case Start_Ctrl://��ʼ��������
			start=1;
			if_idle=0;
			Ctrl_Cmd();
			break;
			
			case Main_Ctrl://������
			if_idle=1;
			start=1;
			Ctrl_Cmd();
			break;
			
			case Stop_Ctrl:// ֹͣ
			if_idle=0;
			start=1;
			Ctrl_Cmd();
			break;
			
			case Jump_Ctrl:// ��Ծ
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
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //����жϱ�־λ
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


