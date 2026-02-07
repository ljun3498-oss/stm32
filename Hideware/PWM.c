#include "stm32f10x.h"                  // Device header
#include "PWM.h"

static void PWM_InitGPIO(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void PWM_InitTimeBase(TIM_TypeDef *TIMx)
{
    TIM_InternalClockConfig(TIMx);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIOD_TICKS;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStructure);
}

static void PWM_InitOC123(TIM_TypeDef *TIMx)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIMx, &TIM_OCInitStructure);
    TIM_OC2Init(TIMx, &TIM_OCInitStructure);
    TIM_OC3Init(TIMx, &TIM_OCInitStructure);
}

void PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    PWM_InitGPIO();

    PWM_InitTimeBase(TIM2);

    PWM_InitOC123(TIM2);

    TIM_Cmd(TIM2, ENABLE);
}

void PWM_Setcompare1(uint16_t Compare)
{
    TIM_SetCompare1(TIM2, Compare);
}

void PWM_Setcompare2(uint16_t Compare)
{
    TIM_SetCompare2(TIM2, Compare);
}

void PWM_Setcompare3(uint16_t Compare)
{
    TIM_SetCompare3(TIM2, Compare);
}
