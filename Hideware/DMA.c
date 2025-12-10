#include "stm32f10x.h"                  // Device header

uint16_t MyDMA_Size;
void MyDMA_Init(uint32_t AddrA,uint32_t AddrB,uint16_t Size)
{
    MyDMA_Size=Size;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
      DMA_InitTypeDef DMA_InitStructure;
    
     DMA_InitStructure.DMA_BufferSize=Size;          //数据量1-65535
     DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;//数据传输方向   dst存储到外设   src外设到存储
     DMA_InitStructure.DMA_M2M=DMA_M2M_Enable;       //是否启用内存到内存
     DMA_InitStructure.DMA_Mode=DMA_Mode_Normal;     //循环模式和普通单次模式 
     DMA_InitStructure.DMA_Priority=DMA_Priority_Medium;//多个通达的优先次序
    
     DMA_InitStructure.DMA_MemoryBaseAddr=AddrB;        //存储基地址
     DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;//数据大小
     DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;                    //优先级
    

     DMA_InitStructure.DMA_PeripheralBaseAddr=AddrA;
     DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
     DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Enable;
    
     DMA_Init(DMA1_Channel1, &DMA_InitStructure);
     
     DMA_Cmd(DMA1_Channel1, DISABLE);//使能
     
}

void MyDMA_Transfer(void)
{
	DMA_Cmd(DMA1_Channel1, DISABLE);					//DMA失能，在写入传输计数器之前，需要DMA暂停工作
	DMA_SetCurrDataCounter(DMA1_Channel1, MyDMA_Size);	//写入传输计数器，指定将要转运的次数
	DMA_Cmd(DMA1_Channel1, ENABLE);						//DMA使能，开始工作
	
	while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);	//等待DMA工作完成
	DMA_ClearFlag(DMA1_FLAG_TC1);						//清除工作完成标志位
}
