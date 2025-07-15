#include "led.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

//LED IO初始化
void ledInit(void)
{    	 
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟
    //GPIOF9,F10初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化

    GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10设置高，灯灭
}
void ledOn(u8 led) // 0 表示pin9 1表示pin10
{
    if(led == 0)
    {
        GPIO_ResetBits(GPIOF, GPIO_Pin_9); //LED0亮
    }
    else if(led == 1)
    {
        GPIO_ResetBits(GPIOF, GPIO_Pin_10); //LED1亮
    }
}
void ledOff(u8 led) // 0 表示pin9 1表示pin10
{
    if(led == 0)
    {
        GPIO_SetBits(GPIOF, GPIO_Pin_9); //LED0灭
    }
    else if(led == 1)
    {
        GPIO_SetBits(GPIOF, GPIO_Pin_10); //LED1灭
    }
}






