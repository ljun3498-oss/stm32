#include "stm32f10x.h"                  // Device header
//红外编码器计次
uint16_t CountSensor_Count;


void CountSensor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO ,ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);//配置中断的gpio口引脚
    
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);//AFIO外部中断引脚选选择配置,AFIO复用引脚一个引脚不够用
    
   
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line=EXTI_Line14;//exti中第十四个线路配置为中断模式
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;//开启触发
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt ;//选择exti为中断模式,有中断和触发两种，中断是内部的需要中断函数触发，事件模式需要外部硬件触发
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//下降电压触发
    EXTI_Init(& EXTI_InitStructure);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;//中断通道编号，有exit  usar tim等等中断编号
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
     NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级
    NVIC_Init(&NVIC_InitStructure);
}
    //外部信号丛io-aifo-exit-nvic-cpu
//aifo是io口复用把某个口的功能调转到接口上
//exti外部中断控制器，配置中断或者事件触发形式以及时候是否触发
//nvic判断各个中断优先级
//两位抢占两位响应，抢占打断当前执行，响应多个中断排序
//通过外部中断来计光电传感器在屏幕上记录次数

uint16_t CountSensor_Get(void)
{
    return CountSensor_Count;
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line14) == SET)
    {
        CountSensor_Count++;
        EXTI_ClearITPendingBit(EXTI_Line14);
    }
}
