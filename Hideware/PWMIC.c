#include "stm32f10x.h"                  // Device header
//输入捕获的pwm波发生器



void PWMIC_Init(void)
{
    //初始化时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    
    //初始化io口
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    //初始化定时器的时间基准单元
    TIM_InternalClockConfig(TIM2);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1 ;
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up ;
    TIM_TimeBaseInitStructure.TIM_Period=100-1;                     //计数周期，即ARR的值
    TIM_TimeBaseInitStructure.TIM_Prescaler=720-1;                  //预分频器，即PSC的值
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
    
    //定时器输出捕获单元（比较器）
    TIM_OCInitTypeDef PWM_FOR_TIM;
    TIM_OCStructInit(&PWM_FOR_TIM);
    //PWM_FOR_TIM.TIM_OCIdleState
    //PWM_FOR_TIM.TIM_OCNIdleState俩高级定时器用的暂时没啥用
    PWM_FOR_TIM.TIM_OCMode=TIM_OCMode_PWM1;
    PWM_FOR_TIM.TIM_OCNPolarity=TIM_OCPolarity_High;
    PWM_FOR_TIM.TIM_OutputState=TIM_OutputNState_Enable;
    PWM_FOR_TIM.TIM_Pulse=0;
    TIM_OC1Init(TIM2,&PWM_FOR_TIM);
    
    TIM_Cmd(TIM2,ENABLE);
}          

void PWM_SetPrescaler(uint16_t Prescaler)
{
    TIM_PrescalerConfig(TIM2, Prescaler, TIM_PSCReloadMode_Immediate);//设置psc的值也就是上面定时器时间基准单元中的预分频TIM_Prescaler
}

void PWM_SetCompare1(uint16_t compare)
{
    TIM_SetCompare1(TIM2,compare);//设置ccr的值也就是上面TIM_Pulse
}
/*




*/
