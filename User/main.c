#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "LED.H"
#include "Key.H"
#include "OLED.H"
#include "CountSensor.H"
#include "Encoder.H"
#include "Timer.H"
#include "PWM.h"
#include "PWMIC.h"
#include "InputCompare.h"
#include "DMA_ADC.H"

//git 到vscode测试1

int main(void)
{
	/*模块初始化*/
	OLED_Init();				//OLED初始化
	DMAAD_Init();					//AD初始化
	
	/*显示静态字符串*/
	OLED_ShowString(1, 1, "AD0:");
	OLED_ShowString(2, 1, "AD1:");
	OLED_ShowString(3, 1, "AD2:");
	OLED_ShowString(4, 1, "AD3:");

	
	while (1)
	{
		OLED_ShowNum(1, 5, AD_Value[0], 4);		//显示转换结果第0个数据
		OLED_ShowNum(2, 5, AD_Value[1], 4);		//显示转换结果第1个数据
		OLED_ShowNum(3, 5, AD_Value[2], 4);		//显示转换结果第2个数据
		OLED_ShowNum(4, 5, AD_Value[3], 4);		//显示转换结果第3个数据

		
		
		Delay_ms(100);							//延时100ms，手动增加一些转换的间隔时间
	}
}



/*
  uint16_t ADValue;			
    float Voltage;	
   int main(void)
{ 
    
    
    
    
    OLED_Init();		//OLED初始化
	PWMIC_Init();			//PWM初始化
    
    
    //adc读取电压值大小
    AD_Init();
    
  	
    
    OLED_ShowString(1, 1, "ADValue:");
	OLED_ShowString(2, 1, "Voltage:0.00V");
    
    while (1)
    {
        ADValue=AD_Getvalue();
        Voltage=(float)ADValue/4095*3.3;
        
        OLED_ShowNum(1, 9, ADValue, 4);				//显示AD值
		OLED_ShowNum(2, 9, Voltage, 1);				//显示电压值的整数部分
		OLED_ShowNum(2, 11, (uint16_t)(Voltage * 100) % 100, 2);	//显示电压值的小数部分
		
		Delay_ms(100);			//延时100ms，手动增加一些转换的间隔时间
    }
	
}
*/

/*
InputCompare();			//输入捕获初始化
    
    //输出pwm
	PWM_SetPrescaler(720-1);
    PWM_SetCompare1(50);
    
    OLED_ShowString(1, 1, "Freq:00000Hz");		//1行1列显示字符串Freq:00000Hz
 
    //输入捕获
    while (1)
    {
      OLED_ShowNum(1, 6, IC_GetFreq(), 5);	//不断刷新显示输入捕获测得的频率
    }


*/
/* while(1)
    {
       for (i=0;i<=100;i++)
        {
            PWM_Setcompare1(i);
            Delay_ms(10);
            
        }
        
        for (i=0;i<=100;i++)
        {
            PWM_Setcompare1(100-i);// 设置ccr寄存器1的大小
            Delay_ms(10);
            
        }
    }
*/
/*
void TIM2_IRQHandler(void)//定时器中断请求
    {
        if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
        {
            Num ++;
            TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
        }
    }
*/

/*OLED_Init();
  CountSensor_Init();
  OLED_ShowString(1,1,"Count:");
   while (1)
   {
        OLED_ShowNum(1,7,CountSensor_Get(),5);
   } 编码器触发中断函数并在OLED屏幕上计数*/


/*RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能时钟-开启a0口的时钟
	
	GPIO_InitTypeDef GPIO_Initstructure;//局部变量定义 定义结构体数据名称是后面俩
	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_Out_PP;//gpio输出模式推免输出
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_0; //gpio引脚
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;//速度
	GPIO_Init(GPIOA,&GPIO_Initstructure);
		
	GPIO_SetBits(GPIOA,GPIO_Pin_0);//io口高电位*/
	
	/*控制寄存器点灯
	Rcc->APB2ENR=0X00000010;
	GPIOC->CRH=0x00300000;
	GPIOC->ODR=0X00002000;
	*/
	/*标准库点灯
	RCC_APB2PeriphClockCmd(USE_STDPERIPH_DRIVER,ENABLE,);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13 
	GPIO_InitStructure.GPIO_Speed=PIO_Speed_50MHz
	GPIO_Init(GPIOc,&GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	GPIO_RestBits(GPIO,GPIO_pin_13);
	*/
	
	/*延时闪烁
		GPIO_WriteBit(GPIOA,GPIO_Pin0,Bit_RESET);
		Delay_ms(500);
		GPIO_WriteBit(GPIOA,GPIO_Pin0,Bit_SET);
		Delay_ms(500);
	 */
	
	/*流水灯
	 GPIO_Write(GPIOA,~0X0001);
		Dleay_ms(500);
			GPIO_Write(GPIOA,~0X0002);
		Dleay_ms(500);
			GPIO_Write(GPIOA,~0X0003);
		Dleay_ms(500);
			GPIO_Write(GPIOA,~0X0004);
		Dleay_ms(500);
			GPIO_Write(GPIOA,~0X0005);
		Dleay_ms(500);
			GPIO_Write(GPIOA,~0X0006);
		Dleay_ms(500);
			GPIO_Write(GPIOA,~0X0007);
		Dleay_ms(500);
		*/
		/*
	while (1)
	{
        LED_Init();
		  GPIO_Write(GPIOA,~0X0001);
		Delay_ms(500);
			GPIO_Write(GPIOA,~0X0002);
		Delay_ms(500);
			GPIO_Write(GPIOA,~0X0003);
		Delay_ms(500);
			GPIO_Write(GPIOA,~0X0004);
		Delay_ms(500);
			GPIO_Write(GPIOA,~0X0005);
		Delay_ms(500);
			GPIO_Write(GPIOA,~0X0006);
		Delay_ms(500);
			GPIO_Write(GPIOA,~0X0007);
		Delay_ms(500);
	}*/
    
    
    //按键控制灯长亮
    /*  uint8_t Keynum;
    Key_Init();
    LED_Init();
    
    while (1)
    {
        Keynum=Key_GetNum();        
       if (Keynum==1) 
       {
           LED1_Turn();
       }
       if(Keynum==2)
        {
            LED2_Turn();
        }
     
    }*/




