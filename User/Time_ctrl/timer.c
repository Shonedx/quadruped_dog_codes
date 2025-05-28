

#include "Allheaderfile.h"
#include "ticks.h"
#include "timer.h"
double now_time=0;
const double dt=0.007; //0.007

 static void Update_Time(void) //获取当前时间，通过自增一个dt值来自增时间
 {
		now_time+=dt;
		now_time=fmod(now_time,1000000);
 }
//Ft=168Mhz/4*时钟分频

void TIM4_Init(void) //10 ms
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 使能 TIM4 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // 2. 配置 TIM4 实现 1ms 中断
    //    假设 TIM4 时钟 (TIMxCLK) = 84MHz (PCLK1=42MHz, APB1 预分频器 > 1)
    //    期望的中断频率 = 1ms (1000Hz)
    //    计算公式：(预分频器 + 1) * (周期 + 1) = TIM4_时钟频率 / 期望频率
    //    (预分频器 + 1) * (周期 + 1) = 84,000,000 Hz / 1000 Hz = 84000

    //    为了实现 1us (1MHz) 的计数器步进：
    //    预分频器 + 1 = 84,000,000 / 1,000,000 = 84  => 预分频器 = 83
    //    然后，为了实现 1ms (1000us) 的中断：
    //    周期 + 1 = 1000 => 周期 = 999

    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;   
    TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;   
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
void TIM5_Init(void) //5 ms
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=5000-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM5,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM4_IRQHandler() //控制类逻辑
{  
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) !=RESET)
	{
		Remote_Cmd();
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}
static int start_stand_flag=0;

void TIM5_IRQHandler(void) //更新时间 5ms
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET)
	{		
		Update_Time();
		if(start==1)
		{		
//			if (currentstate != Jump&& ctrl_state!=Start_Ctrl)
//			 {
//				 IMU_Pos_Cal(0, Euler.pitch, Euler.roll);
//				 Set_Standheight_Offset();
//			 }
			Gait(now_time); //执行步态逻辑
		}
		feed_dog();
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //清除中断标志位
	}
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


