
#include "Allheaderfile.h"
#include "ticks.h"
#include "timer.h"
#include "nrf24l01.h"
#include "RC.h"
#include "led.h"
#include "jump.h"
#include "sys.h"
extern uint8_t rx_buffer[NRF_PAYLOAD_LENGTH];
extern int start;
extern CtrlState_t ctrl_state;
uint64_t timer=0;
 static void Update_Time(void) //更新时间
 {
	    timer++;
 }

void TIM4_Init(void) //1 ms
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;   
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;   
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure); 

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM4, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;             
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;     
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);
}
void TIM5_Init(void) //5 ms
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=2000-1;  
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM5,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM4_IRQHandler() 
{  
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) !=RESET)
	{
		Update_Time();
		NRF24L01_RxPacket_IRQ(rx_buffer);

		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}
extern JumpState_t jump_state;
extern JumpParameter_t jump_structure;

void TIM5_IRQHandler(void) 
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET)
	{		
		if(start==1)
		{		
////			if (currentstate != Jump&& ctrl_state!=Start_Ctrl)
////			 {
////				 IMU_Pos_Cal(0, Euler.pitch, Euler.roll);
////				 Set_Standheight_Offset();
////			 }
			if(ctrl_state!=CS_PRE_JUMP && ctrl_state!=CS_EXE_JUMP)
			{	
				Gait(timer/1000.0f);
				jump_state=IDLE;
			}
			else
				jumpCtrl(timer/1000.0f,&jump_structure);
		}
		else if(start==0&&ctrl_state==CS_QUIT)
		{
			for(int i=0;i<8;i++)
			{
				SetZeroToCanBuf(i);
			}
			Can1_Send_Msg_to_Motor();
			Can2_Send_Msg_to_Motor();
		}
		
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update); 
	}
}
extern int feed; 

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


