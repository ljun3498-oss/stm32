#include "stm32f10x.h"                  // Device header
//输入捕获

void InputCompare(void)
{
    /*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//开启TIM3的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);							//将PA6引脚初始化为上拉输入
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM3);		//选择TIM3为内部时钟，若不调用此函数，TIM默认也为内部时钟
	
	//时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;               //计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;               //预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元
    
    TIM_ICInitTypeDef TIMInitStructure;
    TIMInitStructure.TIM_Channel=TIM_Channel_1 ;
    TIMInitStructure.TIM_ICFilter=0x0 ;
    TIMInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising ;
    TIMInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
    TIMInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //输入信号交叉，选择直通，不交叉
    TIM_ICInit(TIM3,&TIMInitStructure);
    
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);					//触发源选择TI1FP1
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);                 //从模式选择复位
																	//即TI1产生上升沿时，会触发CNT归零
    TIM_Cmd(TIM3,ENABLE);
}

uint32_t IC_GetFreq(void)
{
    return 100000/(TIM_GetCapture1(TIM3)+1);//获得捕获比较器CCR1的值并计算频率
}


/*  每次输入捕获上升沿时，TIM3 计数器 CNT 值被锁存到 CCR1。
定时器工作在“复位从模式”，每检测到一个上升沿就自动清零计数器。
因此 CCR1 中的值即为 信号的周期计数值。
    */
