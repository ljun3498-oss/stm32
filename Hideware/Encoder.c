#include "stm32f10x.h"                  // Device header
int16_t Encoder_Count;

/**
  * @brief  初始化编码器接口及相关中断
  * @param  无
  * @retval 无
  */
void Encoder_Init(void)
{
    // 使能GPIOB和AFIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO ,ENABLE);
    
    // 配置GPIOB的Pin0和Pin1为上拉输入模式
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);//配置中断的gpio口引脚
    
    // 配置GPIOB的Pin0和Pin1为外部中断源
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);//AFIO外部中断引脚选择配置,AFIO复用引脚一个引脚不够用
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);
    
    // 配置外部中断线0和1为中断模式，下降沿触发
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line=EXTI_Line0|EXTI_Line1;//exti中第十四个线路配置为中断模式
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;//开启触发
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt ;//选择exti为中断模式,有中断和触发两种，中断是内部的需要中断函数触发，事件模式需要外部硬件触发
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//下降电压触发
    EXTI_Init(& EXTI_InitStructure);
    
    // 配置中断优先级分组：2位抢占优先级，2位响应优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//两位抢占两位响应，抢占打断当前执行，响应多个中断排序
    
     // 配置外部中断0的NVIC参数
     NVIC_InitTypeDef NVIC_InitStructure;
     NVIC_InitStructure.NVIC_IRQChannel=EXTI0_IRQn;//中断通道编号，有exit  usar tim等等中断编号
     NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
     NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级
     NVIC_Init(&NVIC_InitStructure);
    
     // 配置外部中断1的NVIC参数
     NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn;//中断通道编号，有exit  usar tim等等中断编号
     NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
     NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;//响应优先级
     NVIC_Init(&NVIC_InitStructure);
}


void EXTI0_IRQHandler(void)
{
    
    if (EXTI_GetITStatus(EXTI_Line0)==SET)
    {
        if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==0)
        {
            Encoder_Count --;
        }
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

int16_t Encoder_Get(void)
{
    int16_t Temp;
    Temp = Encoder_Count;
    Encoder_Count=0;
    return Temp;
    
}
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1)==SET)
    {
          if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==0)
        {
            Encoder_Count ++;
        }
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
