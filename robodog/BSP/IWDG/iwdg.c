#include "iwdg.h"
#include "stm32f4xx_iwdg.h"
#include "led.h"
void iwdgInit(u8 prer,u16 rlr) //prer 500 rlr 4 此时超时时间为=500*4*2^4/32k;
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	IWDG_SetPrescaler(prer); //设置IWDG分频系数
	IWDG_SetReload(rlr);   //设置IWDG装载值
	IWDG_ReloadCounter(); //reload
	IWDG_Enable();       //使能看门狗
}

//喂狗防止死机
void iwdgFeed(void)
{
	IWDG_ReloadCounter();//reload
}
extern int feed; 

void feedDog(void)
{
	if(feed)
	{
		iwdgFeed();	
		ledOn(1); //led pin10亮 灯亮表示正常喂狗
	}
	else
	{
		ledOff(1);
	}

}
