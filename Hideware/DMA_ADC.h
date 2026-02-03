#ifndef __DMA_ADC_H
#define __DMA_ADC_H

#include "stm32f10x.h"                  // Device header

/**
 * @brief  ADC转换结果数组
 * @note   存储4个ADC通道的转换结果，对应PA0-PA3
 */
extern uint16_t AD_Value[4];

/**
 * @brief  DMA_ADC初始化函数
 * @note   配置ADC1和DMA1，实现多通道ADC数据的自动采集和存储
 * @param  无
 * @retval 无
 */
void DMAAD_Init(void);

#endif
