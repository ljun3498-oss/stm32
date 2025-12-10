#include "stm32f10x.h"                  // Device header

extern uint16_t Num;//跨文件使用已经定义变量

/*定时器开启步骤
1.开启时钟
2.时钟基准单元输入时钟（内/外模式等）
3.配置时钟基准单元
4.配置中断输出控制类似exti
5.配置nvic配置中断优先级
6.计数器使能
*/


void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//tim时钟挂在apb1总线
    
    TIM_InternalClockConfig(TIM2);//使用内部时钟
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//时钟基准单元配置函数下面是具体参数
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1 ;//指定时钟分频1分频
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up ;//计数模式向上计数
    TIM_TimeBaseInitStructure.TIM_Period=10000-1;//ARR计数到多少溢出
    TIM_TimeBaseInitStructure.TIM_Prescaler=7200-1;//PSC预分频器：7200分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重复计数 高级定时器才有
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//应用配置
    
    TIM_ClearFlag(TIM2,TIM_FLAG_Update);//清除更新中断标志位（中）
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//  使能TIM2的更新中断（计数溢出时产生中断）
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIM2,ENABLE);
}


/*void TIM2_IRQHandler(void)
    {
        if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
        {
            Num ++;
            TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
        }
    }
*/
