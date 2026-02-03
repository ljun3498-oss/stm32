#include "stm32f10x.h"                  // Device header

// 存储ADC转换结果的数组，共4个通道
uint16_t AD_Value[4];


/**
 * @brief  DMA_ADC初始化函数
 * @note   配置ADC1和DMA1，实现多通道ADC数据的自动采集和存储
 * @param  无
 * @retval 无
 */
void DMAAD_Init(void)
{
	// 开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	// 开启ADC1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	// 开启GPIOA时钟（ADC通道所在端口）
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		// 开启DMA1时钟
	
	// 设置ADC时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);	// 设置ADC时钟为PCLK2的6分频，即72MHz/6=12MHz
	
	// GPIO初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	// 设置为模拟输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;	// 配置PA0-PA3四个通道
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// 引脚速度（模拟输入模式下无实际意义）
	GPIO_Init(GPIOA, &GPIO_InitStructure);				
	
	// ADC配置
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;		// 启用连续转换模式
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	// 数据右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	// 不使用外部触发，使用软件触发
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;		// ADC1独立工作模式
	ADC_InitStructure.ADC_NbrOfChannel = 4;				// 转换通道数量为4个
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			// 启用扫描模式（多通道转换需要）
	ADC_Init(ADC1, &ADC_InitStructure);	
	
	// DMA初始化
	DMA_InitTypeDef DMA_InitStructure;	
	DMA_InitStructure.DMA_BufferSize = 4;				// 缓冲区大小，与通道数对应
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	// 数据传输方向：外设到内存（ADC到内存）
	DMA_InitStructure.DMA_M2M = DISABLE;				// 禁用内存到内存传输
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		// 启用循环模式，自动重复传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;	// DMA通道优先级为中等
	
	// 内存配置
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;	// 内存基地址，指向AD_Value数组
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;	// 内存数据宽度：半字(16位)
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	// 内存地址自增，指向下一个存储位置

	// 外设配置
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;	// 外设基地址，指向ADC1的数据寄存器
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	// 外设数据宽度：半字(16位)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	// 外设地址不自增（始终使用同一个寄存器）
	
	// 应用DMA配置到通道1（ADC1对应DMA1的通道1）
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
		
	DMA_Cmd(DMA1_Channel1, ENABLE);		// 启用DMA1的通道1
	ADC_DMACmd(ADC1, ENABLE);			// 启用ADC1的DMA请求
	ADC_Cmd(ADC1, ENABLE);				// 启用ADC1
	
	// ADC校准流程（固定步骤）
	ADC_ResetCalibration(ADC1);								// 重置校准寄存器
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);		// 等待校准重置完成
	ADC_StartCalibration(ADC1);								// 开始校准
	while (ADC_GetCalibrationStatus(ADC1) == SET);			// 等待校准完成
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	// 软件触发ADC转换开始
}