#include "stm32f10x.h"                  // Device header

void AD_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    GPIO_InitTypeDef GPIO_InitStructure;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   	GPIO_Init(GPIOA, &GPIO_InitStructure);	
    
    ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);
    
    ADC_InitTypeDef ADC_InitSructure;
    ADC_InitSructure.ADC_DataAlign=ADC_DataAlign_Right ;
    ADC_InitSructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;//使用软件触发不需要外部触发
    ADC_InitSructure.ADC_Mode=ADC_Mode_Independent;
    ADC_InitSructure.ADC_NbrOfChannel=1;//通道数量非扫描模式下只有一个最大16
    ADC_InitSructure.ADC_ScanConvMode=DISABLE;//是否扫描模式
    ADC_InitSructure.ADC_ContinuousConvMode=ENABLE;//是否连续转换
    ADC_Init(ADC1,&ADC_InitSructure);
	  ADC_Cmd(ADC1, ENABLE);
    
    ADC_ResetCalibration(ADC1);//adc转换校准的固定流程，内部有电路会自动执行校准
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);//
	while (ADC_GetCalibrationStatus(ADC1) == SET);
    
}




uint16_t AD_Getvalue(void)
{
    ADC_SoftwareStartConvCmd(ADC1,ENABLE);//开启adc
    while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);//检查adc是否转换完成
    return ADC_GetConversionValue(ADC1);//返回adc转换的值
}
