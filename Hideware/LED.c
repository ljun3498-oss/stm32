#include "stm32f10x.h"                  // Device header
//led灯的端口初始化程序

void LED_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
}
//初始化led灯端口时钟
 void LED1_ON(void)
    {
        GPIO_ResetBits(GPIOA,GPIO_Pin_1);
    }	
    void LED1_OFF(void)
    {
        GPIO_ResetBits(GPIOA,GPIO_Pin_1);
    }	
    void LED2_ON(void)
    {
        GPIO_ResetBits(GPIOA,GPIO_Pin_2);
    }	
    void LED2_OFF(void)
    {
        GPIO_ResetBits(GPIOA,GPIO_Pin_2);
    }	
    //led12两个端口电位高低

    void LED1_Turn(void)
   {
        if (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1)==0)
        {
            GPIO_SetBits(GPIOA,GPIO_Pin_1);
        }
        else
        {
            GPIO_ResetBits(GPIOA,GPIO_Pin_1);
        }
    }
        
         void LED2_Turn(void)
    {
        
        if (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_11)==0)
        {
            GPIO_SetBits(GPIOA,GPIO_Pin_11);
        }
        else
        {
            GPIO_ResetBits(GPIOA,GPIO_Pin_11);
        }
    }//IO口翻转
    
    
