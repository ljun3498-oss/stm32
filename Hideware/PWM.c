#include "stm32f10x.h"                  // Device header


void PWM_Init(void)
{
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//tim时钟挂在apb1总线
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开启引脚重映射时钟
    
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);//引脚重映射
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//把15端口原来的swj调试关掉改gpio的功能才能把定时器从pa0移动到pa15
   
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//设置复用推免输才能将io口从输出数据寄存器断开转道片上外设的复用
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    TIM_InternalClockConfig(TIM2);//使用内部时钟
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//时钟基准单元配置函数下面是具体参数
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1 ;//指定时钟分频1分频
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up ;//计数模式向上计数
    TIM_TimeBaseInitStructure.TIM_Period=100-1;//ARR计数到多少溢出
    TIM_TimeBaseInitStructure.TIM_Prescaler=720-1;//PSC预分频器：7200分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重复计数 高级定时器才有
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//应用配置
    
    TIM_ClearFlag(TIM2,TIM_FLAG_Update);//清除更新中断标志位（中）
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//  使能TIM2的更新中断（计数溢出时产生中断）
    
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);//对所有输出捕获寄存定义初始值
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1 ;//8种输出比较模式定义一个
    TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//输出急性
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//输出状态 使能
    TIM_OCInitStructure.TIM_Pulse=0; //ccr寄存器值0-ffff之间 下面有tim—setcompare可以设置ccr的值
    TIM_OC1Init(TIM2,&TIM_OCInitStructure);
    
    TIM_Cmd(TIM2,ENABLE);
}


void PWM_Setcompare1(uint16_t Compare)
{
    TIM_SetCompare1(TIM2,Compare);
}
