#include "stm32f10x.h"                  // Device header


uint16_t AD_Value[4];


void DMAAD_Init(void)
{
	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//adc1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//gpioa时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		//dma1时钟
	
	//设置adc时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//设置adc时钟6分频  72/6
	
	//GPIO初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//设置为模拟输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);				
	
	//adc组配置,配置工作模式，扫描模式转换模式，输入通道等
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_ContinuousConvMode=ENABLE;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel=4;
	ADC_InitStructure.ADC_ScanConvMode=ENABLE;
	ADC_Init(ADC1, &ADC_InitStructure);	
	
	//dma初始化
	DMA_InitTypeDef DMA_InitStructure;	
	DMA_InitStructure.DMA_BufferSize=4;
	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;//外设到存储器还是存储器到外设
	DMA_InitStructure.DMA_M2M=DISABLE;//是否内存到内存
	DMA_InitStructure.DMA_Mode=DMA_Mode_Circular;//是否循环
	DMA_InitStructure.DMA_Priority=DMA_Priority_Medium;//优先级
	
	DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)AD_Value;//地址
	DMA_InitStructure.DMA_MemoryDataSize=DMA_PeripheralDataSize_HalfWord;//数据长度
	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;//地址是否自增

	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
		
	DMA_Cmd(DMA1_Channel1, ENABLE);//DMA1的通道1使能
	ADC_DMACmd(ADC1, ENABLE);//ADC1触发DMA1的信号使能
	ADC_Cmd(ADC1, ENABLE);//ADC1使能
	
	
	ADC_ResetCalibration(ADC1);								//固定流程，内部有电路会自动执行校准
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//软件触发

}


