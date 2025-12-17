#include "stm32f10x.h"                  // Device header

uint16_t MyDMA_Size;
/**
  * @brief  初始化DMA传输配置
  * @param  AddrA 源地址
  * @param  AddrB 目的地址
  * @param  Size  传输数据长度（元素个数）
  * @retval 无
  */
void MyDMA_Init(uint32_t AddrA,uint32_t AddrB,uint16_t Size)
{
    MyDMA_Size=Size;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
      DMA_InitTypeDef DMA_InitStructure;
    
    DMA_InitStructure.DMA_BufferSize=Size;                    // 设置传输长度 1-65535
    DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;         // 传输方向：外设 -> 内存（外设为源）
    DMA_InitStructure.DMA_M2M=DMA_M2M_Enable;                // 内存到内存传输使能（根据需要配置）
    DMA_InitStructure.DMA_Mode=DMA_Mode_Normal;              // 工作模式：普通模式（非循环）
    DMA_InitStructure.DMA_Priority=DMA_Priority_Medium;      // 通道优先级：中等

    DMA_InitStructure.DMA_MemoryBaseAddr=AddrB;              // 内存基地址
    DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte; // 内存数据宽度：字节
    DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;    // 内存地址自增使能

    DMA_InitStructure.DMA_PeripheralBaseAddr=AddrA;         // 外设基地址
    DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte; // 外设数据宽度：字节
    DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Enable; // 外设地址自增使能
    
     DMA_Init(DMA1_Channel1, &DMA_InitStructure);
     
    DMA_Cmd(DMA1_Channel1, DISABLE); // 先禁用DMA，完成配置后再启用
     
}

void MyDMA_Transfer(void)
{
  DMA_Cmd(DMA1_Channel1, DISABLE);                    // 先禁用DMA，准备重新设置传输计数
  DMA_SetCurrDataCounter(DMA1_Channel1, MyDMA_Size); // 设置当前传输数据计数
  DMA_Cmd(DMA1_Channel1, ENABLE);                     // 启用DMA，开始传输

  while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET); // 等待传输完成
  DMA_ClearFlag(DMA1_FLAG_TC1);                      // 清除传输完成标志
}
