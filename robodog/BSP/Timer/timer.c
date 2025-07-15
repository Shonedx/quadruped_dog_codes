#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "timer.h"
#include "rc_spi.h"
#include "RC.h"
#include "rc_nrf24l01.h"
#include "motion_ctrl.h"
#include "can.h"
#include "main.h" //start 变量
volatile uint64_t timer=0; //加volatile修饰符，防止编译器优化
extern u8 rx_buffer[NRF_PAYLOAD_LENGTH];
extern volatile u8 start;
extern JumpState_t jump_state;

// extern CtrlState_t ctrl_state;

 static void updateTimer(void) //获取当前时间，通过定时中断不断计时
 {
		timer++;
 }
void tim1Init(void)// 0.1ms
{
	TIM_TimeBaseInitTypeDef tim_base_init_structure;
    NVIC_InitTypeDef nvic_init_structure;
	//rcc启动时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //使能TIM1时钟
	//时基单元
	tim_base_init_structure.TIM_Prescaler = 84 - 1; //预分
	tim_base_init_structure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	tim_base_init_structure.TIM_Period = 100 - 1; //自动重装载值
	tim_base_init_structure.TIM_ClockDivision = TIM_CKD_DIV1; //无时钟分频
	TIM_TimeBaseInit(TIM1, &tim_base_init_structure); //初始化TIM1	
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能TIM1更新中断
	TIM_Cmd(TIM1, ENABLE); //启动定时器
	//nvic
	nvic_init_structure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn; //TIM1更新中断
	nvic_init_structure.NVIC_IRQChannelPreemptionPriority = 0; //抢占
	nvic_init_structure.NVIC_IRQChannelSubPriority = 0; //子优先级
	nvic_init_structure.NVIC_IRQChannelCmd = ENABLE; //使能中断通道
	NVIC_Init(&nvic_init_structure); //初始化NVIC

}

 //原时钟频率 84MHz
void tim4Init(void) //1 ms
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 使能 TIM4 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    // 2. 配置 TIM4 时间基准
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;   
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;   
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 无时钟分频
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure); // 初始化 TIM4
    // 3. 使能 TIM4 更新中断
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    // 4. 启动定时器
    TIM_Cmd(TIM4, ENABLE);
    // 5. 配置 NVIC 以启用 TIM4 中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;             // TIM4 全局中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);
}
void tim5Init(void) //5 ms
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//初始化时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	//配置时基单元
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=5000-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	//使能更新中断
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	//启动定时器
	TIM_Cmd(TIM5,ENABLE);
	//配置nvic启动tim5中断
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM1_UP_TIM10_IRQHandler(void) //0.1 ms
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
		updateTimer();
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
void TIM4_IRQHandler() //接收端 1 ms
{  
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) !=RESET)
	{
		
		nrfRxPacketIrq(rx_buffer);
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}
void TIM5_IRQHandler(void) //控制端5ms
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
				gaitCtrl(timer/10000.0f); //执行步态逻辑
				jump_state=IDLE;
			}
			else
				jumpCtrl();
		}
		else if(start==0&&ctrl_state==CS_QUIT)
		{
			for(int i=0;i<8;i++)
			{
				setZeroToCanBuf(i);
			}
			txMsgToMotor(0);//发送数据到can1的电机
			txMsgToMotor(1); //发送数据到can2的电机
		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //清除中断标志位
	}
}


