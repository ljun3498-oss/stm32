#ifndef __DMA_ADC_H
#define __DMA_ADC_H

#include "stm32f10x.h"                  // Device header

/**
 * @brief  ADC转换结果数组
 * @note   存储3个ADC通道的转换结果，对应PA5-PA7
 */
extern uint16_t AD_Value[3];

/**
 * @brief  DMA_ADC初始化函数
 * @note   配置ADC1和DMA1，实现多通道ADC数据的自动采集和存储
 * @param  无
 * @retval 无
 */
void DMAAD_Init(void);

/**
 * @brief  将ADC值转换为电流（毫安）
 * @note   0A=2.5V, 1A=0.185V
 */
int32_t DMAAD_AdcToCurrent_mA(uint16_t adc);

#endif

