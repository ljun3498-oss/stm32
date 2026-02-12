#include "stm32f10x.h"
#include "delay.h"
#include "OLED.H"
#include <stddef.h>

/* ==================== Global Variables ==================== */

// PWM Module Constants
#define PWM_FREQ_KHZ       20
#define PWM_TIMER_CLK_HZ   72000000
#define PWM_CENTER_ALIGNED 1
#if PWM_CENTER_ALIGNED
#define PWM_PERIOD_TICKS   ((PWM_TIMER_CLK_HZ / (PWM_FREQ_KHZ * 1000U * 2U)) - 1U)
#else
#define PWM_PERIOD_TICKS   ((PWM_TIMER_CLK_HZ / (PWM_FREQ_KHZ * 1000U)) - 1U)
#endif
#define PWM_PERIOD_TOTAL   (PWM_PERIOD_TICKS + 1U)

// ADC Module Constants
#define ADC_SAMPLE_RATE_HZ     10000U
#define ADC_TIMER_CLK_HZ       72000000U

// Encoder Module Constants
#define MT6701_I2C_ADDR        0x06
#define MT6701_REG_ANGLE_MSB   0x03
#define MT6701_ELEC_COUNTS     16384
#define MT6701_POLE_PAIRS      7          // 电机极对数：7对
#define MT6701_MECH_COUNTS     (MT6701_ELEC_COUNTS * 4)  // 机械计数 = 编码器4圈 = 电机1圈（4:1传动比）
#define BUFFER_SIZE            32

// FOC Module Constants
#define Q15_SCALE       32768
#define Q15_MULT(a, b)  ((int32_t)(a) * (int32_t)(b) >> 15)
#define ANGLE_MAX       65536
#define DEG_TO_ANGLE(deg)  ((uint16_t)((deg) * 65536L / 360))
#define BUS_VOLTAGE_MV  12000        // 母线电压 12V
#define MAX_CURRENT_MA  1500         // 额定电流 1.5A
#define MAX_CURRENT_Q15 32767        // 1.5A对应的Q15值（最大值）
#define ELEC_ALIGN_SHIFT_TENTHS 0    // 电角度固定偏移(0.1度)，默认0，交由对齐计算
#define ELEC_ANGLE_DIR  -1           // 电角度方向: 1=正向, -1=反向
#define KP_ID_Q15       3277         // Kp=0.1 (适中的响应速度)
#define KI_ID_Q15       200          // Ki=200
#define KP_IQ_Q15       3277         // Kp=0.1
#define KI_IQ_Q15       200          // Ki=200
#define DT_MS           0.05         // 实际控制周期50µs (20kHz)
#define FOC_ISR_HZ      ADC_SAMPLE_RATE_HZ
#define ALIGNMENT_SAMPLES (FOC_ISR_HZ / 1U)  // 1s (加强对齐时间)
#define VOLT_MAX_Q15    26214        // 0.8 in Q15
#define OPEN_LOOP_TEST  0            // 1=开环测试, 0=闭环FOC
#define OPEN_LOOP_VD_Q15 0
#define OPEN_LOOP_VQ_Q15 4500        // 开环q轴电压(~8%，增大以获得更多转矩)
#define OPEN_LOOP_STEP_Q16 14         // 电角度步进(20kHz更新，极低速度用于测试)

// Sin/Cos查找表（256点，Q15格式）
static const int16_t sin_table[256] = {
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
    23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790, 27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
    30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
    32767, 32757, 32728, 32678, 32609, 32521, 32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
    30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790, 26319, 25832, 25329, 24811, 24279, 23731,
    23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868, 18204, 17530, 16846, 16151, 15446, 14732, 14010, 13279,
    12539, 11793, 11039, 10278, 9512, 8739, 7962, 7179, 6393, 5602, 4808, 4011, 3212, 2410, 1608, 804,
    0, -804, -1608, -2410, -3212, -4011, -4808, -5602, -6393, -7179, -7962, -8739, -9512, -10278, -11039, -11793,
    -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790, -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
    -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971, -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
    -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285, -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
    -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868, -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278, -9512, -8739, -7962, -7179, -6393, -5602, -4808, -4011, -3212, -2410, -1608, -804
};

// ADC Module
uint16_t AD_Value[3];
static uint16_t g_adc_zero_offset[3] = {2048, 2048, 2048};  // 默认2.5V为中点
// ADC一阶滤波缓冲（alpha≈1/16，平滑ADC噪声）
static int32_t g_adc_filt[3] = {0, 0, 0};

// Encoder Module (non-buffer variables)
static volatile uint32_t g_ms = 0;
static uint16_t g_last_raw = 0;
static int32_t g_accum = 0;
static int32_t g_last_speed_accum = 0;
static uint32_t g_last_speed_time = 0;
static volatile int16_t g_current_rpm = 0;
static volatile uint8_t g_dma_active = 0;

// Encoder Module (buffer variables)
static uint8_t g_buffer0[BUFFER_SIZE * 2];
static uint8_t g_buffer1[BUFFER_SIZE * 2];
static uint8_t *g_active_buffer = g_buffer0;
static uint8_t *g_ready_buffer = NULL;
static volatile uint16_t g_buffer_index = 0;
static volatile uint8_t g_buffer_ready = 0;

// 编码器原始值缓存（在读取失败时使用）
static uint16_t encoder_raw_cache = 0;
static volatile uint16_t g_encoder_raw_for_foc = 0;  // 供FOC中断用的编码器原始值(14bit)

// 三相电流显示变量
static volatile int16_t g_Ia_mA = 0;
static volatile int16_t g_Ib_mA = 0;
static volatile int16_t g_Ic_mA = 0;

// FOC Module
#define FOC_STATE_IDLE       0   // 空闲，PWM输出0
#define FOC_STATE_ALIGN      1   // 对齐模式，锁定转子到d轴
#define FOC_STATE_OPENLOOP   2   // 开环模式，验证方向和极对数
#define FOC_STATE_CLOSEDLOOP 3   // 闭环FOC模式

static uint16_t g_cached_angle_tenths = 0;  // 缓存当前编码器角度
static volatile uint8_t g_alignment_done = 0;  // 对齐完成标志
static volatile uint8_t g_oled_update_flag = 0;
static volatile uint8_t g_foc_enabled = 0;
static volatile uint8_t g_foc_state = FOC_STATE_IDLE;  // FOC状态机
static volatile uint8_t g_theta_flip = 1;  // 电角度方向翻转（1=反向）- 解决"转一下卡住"
static volatile uint32_t g_align_counter = 0;    // 对齐计数器
static int16_t g_iq_ref_q15 = 0;                 // q轴电流参考 (Q15格式)
static volatile uint16_t g_open_loop_theta = 0;  // 开环电角度（软件步进）

// FOC PI Controllers
static int32_t Id_int_q15 = 0;  // d轴积分（对齐时会自动建立）
static int32_t Iq_int_q15 = 0;  // q轴积分

// OLED Display Variables
static uint16_t display_counter = 0;

/* ==================== PWM Module ==================== */

static void PWM_InitGPIO(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void PWM_InitTimeBase(TIM_TypeDef *TIMx)
{
    TIM_InternalClockConfig(TIMx);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIOD_TICKS;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStructure);
}

static void PWM_InitOC123(TIM_TypeDef *TIMx)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIMx, &TIM_OCInitStructure);
    TIM_OC2Init(TIMx, &TIM_OCInitStructure);
    TIM_OC3Init(TIMx, &TIM_OCInitStructure);
}

void PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    PWM_InitGPIO();
    PWM_InitTimeBase(TIM2);
    PWM_InitOC123(TIM2);

    // 初始化PWM为0，防止上电时电机意外转动
    TIM_SetCompare1(TIM2, 0);
    TIM_SetCompare2(TIM2, 0);
    TIM_SetCompare3(TIM2, 0);

    // 配置TIM2更新中断（最高优先级，用于FOC控制环 @ 20kHz）
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 最高抢占优先级
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    TIM_Cmd(TIM2, ENABLE);
}

void PWM_Setcompare1(uint16_t Compare) { TIM_SetCompare1(TIM2, Compare); }
void PWM_Setcompare2(uint16_t Compare) { TIM_SetCompare2(TIM2, Compare); }
void PWM_Setcompare3(uint16_t Compare) { TIM_SetCompare3(TIM2, Compare); }

/* ==================== ADC Module ==================== */
uint16_t AD_Value[3];

static void DMAAD_TimerTriggerInit(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = (ADC_TIMER_CLK_HZ / ADC_SAMPLE_RATE_HZ) - 1U;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
    // 暂不配置同步模式，TIM3独立运行以保证ADC采样稳定
    // (同步配置可以后续优化，但不应该破坏采样)
    
    // 配置TIM3中断用于FOC电流环（初始禁用，按键启动后才启用）
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 启用TIM3计数器，使其开始运行
    TIM_Cmd(TIM3, ENABLE);
}

void DMAAD_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 先初始化 DMA
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_BufferSize = 3;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M = DISABLE;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    // 初始化 ADC
    ADC_DeInit(ADC1);
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  // 改为软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 3;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // 配置3个通道的扫描序列
    // 增加采样时间以降低噪声：71.5周期（约7μs/通道）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_71Cycles5);
    
    // 启用 DMA 和 ADC
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    
    // ADC 校准
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);
    
    // 初始化定时器触发
    DMAAD_TimerTriggerInit();

    // 启动连续转换（DMA会持续更新AD_Value）
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    // ADC 预热延迟，让 DMA 先收集几个采样点稳定数据
    Delay_ms(200);
}

// ADC零漂补偿（上电时校准）

void DMAAD_CalibrateZeroPoint(void)
{
    // 采用软件触发采样，计算真实零点（电机静止、无负载）
    uint32_t accum[3] = {0, 0, 0};
    uint16_t i;

    for (i = 0; i < 128; i++)
    {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        Delay_ms(1);
        accum[0] += AD_Value[0];
        accum[1] += AD_Value[1];
        accum[2] += AD_Value[2];
    }

    g_adc_zero_offset[0] = (uint16_t)(accum[0] >> 7);
    g_adc_zero_offset[1] = (uint16_t)(accum[1] >> 7);
    g_adc_zero_offset[2] = (uint16_t)(accum[2] >> 7);
}

int32_t DMAAD_AdcToCurrent_mA(uint16_t adc, uint8_t channel)
{
    // 使用校准的零点进行转换
    int16_t adc_zero = (int16_t)g_adc_zero_offset[channel];
    int16_t adc_delta = (int16_t)adc - adc_zero;  // 相对于零点的偏移
    
    // ADC转电压：delta再转电流
    int32_t delta_mv = (int32_t)adc_delta * 3300 / 4095;
    return (delta_mv * 1000) / 185;  // 185mΩ采样电阻
}

/* ==================== Encoder Module ==================== */

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        g_ms++;
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

void DMA1_Channel5_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC5))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC5);

        I2C_GenerateSTOP(I2C2, ENABLE);
        DMA_Cmd(DMA1_Channel5, DISABLE);
        I2C_DMACmd(I2C2, DISABLE);

        g_buffer_index++;
        if (g_buffer_index >= BUFFER_SIZE)
        {
            g_buffer_ready = 1;
            g_ready_buffer = g_active_buffer;
            g_active_buffer = (g_active_buffer == g_buffer0) ? g_buffer1 : g_buffer0;
            g_buffer_index = 0;
        }

        DMA_SetCurrDataCounter(DMA1_Channel5, 2);
        DMA1_Channel5->CMAR = (uint32_t)(g_active_buffer + g_buffer_index * 2);

        I2C_GenerateSTART(I2C2, ENABLE);
        uint32_t timeout = 1000;
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) { timeout--; }

        I2C_Send7bitAddress(I2C2, MT6701_I2C_ADDR << 1, I2C_Direction_Transmitter);
        timeout = 1000;
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout > 0) { timeout--; }

        I2C_SendData(I2C2, MT6701_REG_ANGLE_MSB);
        timeout = 1000;
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout > 0) { timeout--; }

        I2C_GenerateSTART(I2C2, ENABLE);
        timeout = 1000;
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) { timeout--; }

        I2C_Send7bitAddress(I2C2, MT6701_I2C_ADDR << 1, I2C_Direction_Receiver);
        timeout = 1000;
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout > 0) { timeout--; }

        I2C_DMALastTransferCmd(I2C2, ENABLE);
        I2C_DMACmd(I2C2, ENABLE);
        DMA_Cmd(DMA1_Channel5, ENABLE);
    }
}

static void Encoder_TimeBaseInit(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  // 最低优先级（通用计时）
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);
}

static void Encoder_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel5);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(I2C2->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_active_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 2;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  // 中等优先级（编码器DMA）
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

static uint8_t MT6701_ReadRegPair(uint8_t reg, uint8_t *msb, uint8_t *lsb)
{
    uint32_t timeout;
    
    // 禁用TIM2中断，防止I2C事务被打断导致状态机损坏
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    
    timeout = 10000;
    // 等待总线空闲（带超时）
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) && timeout > 0) 
    {
        timeout--;
    }
    
    if (timeout == 0) {
        // 总线卡死，软件复位I2C2
        I2C_SoftwareResetCmd(I2C2, ENABLE);
        I2C_SoftwareResetCmd(I2C2, DISABLE);
        
        I2C_InitTypeDef I2C_InitStructure;
        I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
        I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
        I2C_InitStructure.I2C_OwnAddress1 = 0x00;
        I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
        I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        I2C_InitStructure.I2C_ClockSpeed = 400000;
        I2C_Init(I2C2, &I2C_InitStructure);
        I2C_Cmd(I2C2, ENABLE);
        TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
        return 0;
    }

    I2C_GenerateSTART(I2C2, ENABLE);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) { timeout--; }

    I2C_Send7bitAddress(I2C2, MT6701_I2C_ADDR << 1, I2C_Direction_Transmitter);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout > 0) { timeout--; }

    I2C_SendData(I2C2, reg);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout > 0) { timeout--; }

    I2C_GenerateSTART(I2C2, ENABLE);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) { timeout--; }

    I2C_Send7bitAddress(I2C2, MT6701_I2C_ADDR << 1, I2C_Direction_Receiver);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout > 0) { timeout--; }

    timeout = 10000;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) && timeout > 0) { timeout--; }
    *msb = I2C_ReceiveData(I2C2);

    I2C_AcknowledgeConfig(I2C2, DISABLE);
    timeout = 10000;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) && timeout > 0) { timeout--; }
    *lsb = I2C_ReceiveData(I2C2);

    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    // 恢复TIM2中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    return 1;
}

// 前向声明，使 Encoder_Init() 能调用
uint16_t Encoder_GetRaw14(void);

void Encoder_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;
    I2C_Init(I2C2, &I2C_InitStructure);

    I2C_Cmd(I2C2, ENABLE);

    g_ms = 0;
    g_last_raw = 0;
    g_accum = 0;
    g_last_speed_accum = 0;
    g_last_speed_time = 0;
    g_current_rpm = 0;
    g_buffer_index = 0;
    g_buffer_ready = 0;
    g_dma_active = 0;


    Encoder_TimeBaseInit();
    Encoder_DMA_Init();
    
    // 初始化编码器：预读多次以建立正确的g_last_raw基准
    // 如果编码器有连接，应该能读到不同的值
    Delay_ms(100);
    uint16_t init_val = Encoder_GetRaw14();  // 多读几次，确保I2C工作
    Delay_ms(20);
    init_val = Encoder_GetRaw14();
    Delay_ms(20);
    init_val = Encoder_GetRaw14();
    
    // 记录初始化值
    g_last_raw = init_val;
    encoder_raw_cache = init_val;
    g_accum = (int32_t)init_val;
}

void Encoder_StartHighSpeedAcquisition(void)
{
    if (g_dma_active) return;

    g_buffer_index = 0;
    g_buffer_ready = 0;
    g_dma_active = 1;

    I2C_GenerateSTART(I2C2, ENABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C2, MT6701_I2C_ADDR << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C2, MT6701_REG_ANGLE_MSB);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTART(I2C2, ENABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C2, MT6701_I2C_ADDR << 1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    I2C_DMALastTransferCmd(I2C2, ENABLE);
    I2C_DMACmd(I2C2, ENABLE);
    DMA_Cmd(DMA1_Channel5, ENABLE);
}

uint8_t Encoder_GetReadyBuffer(uint8_t **buffer, uint16_t *size)
{
    if (!g_buffer_ready) return 0;

    *buffer = g_ready_buffer;
    *size = g_buffer_index * 2;
    g_buffer_ready = 0;
    return 1;
}

void Encoder_ProcessBuffer(uint8_t *buffer, uint16_t size)
{
    uint16_t i;
    uint32_t now_ms = g_ms;
    uint32_t dt_ms = now_ms - g_last_speed_time;
    int32_t delta_accum;

    for (i = 0; i < size; i += 2)
    {
        uint16_t raw = (uint16_t)((buffer[i] << 8) | buffer[i + 1]) & 0x3FFF;
        int32_t delta = (int32_t)raw - (int32_t)g_last_raw;

        if (delta > (MT6701_ELEC_COUNTS / 2))
        {
            delta -= MT6701_ELEC_COUNTS;
        }
        else if (delta < -(MT6701_ELEC_COUNTS / 2))
        {
            delta += MT6701_ELEC_COUNTS;
        }

        g_accum -= delta;  // 反向编码器，与Encoder_Get()保持一致
        g_last_raw = raw;
    }

    if (dt_ms > 0)
    {
        delta_accum = g_accum - g_last_speed_accum;
        g_current_rpm = (int16_t)((int64_t)delta_accum * 60000LL / (MT6701_MECH_COUNTS * (int32_t)dt_ms));
        g_last_speed_accum = g_accum;
        g_last_speed_time = now_ms;
    }
}

static void Encoder_UpdateSpeedFromAccum(void)
{
    uint32_t now_ms = g_ms;
    uint32_t dt_ms = now_ms - g_last_speed_time;
    int32_t delta_accum;

    if (dt_ms < 100)
    {
        return;  // 需要累计至少100ms才计算速度
    }

    delta_accum = g_accum - g_last_speed_accum;
    g_current_rpm = (int16_t)((int64_t)delta_accum * 60000LL / (MT6701_MECH_COUNTS * (int32_t)dt_ms));
    g_last_speed_accum = g_accum;
    g_last_speed_time = now_ms;
}

uint32_t Encoder_GetTimeMs(void)
{
    return g_ms;
}

uint16_t Encoder_GetRaw14(void)
{
    uint8_t msb = 0;
    uint8_t lsb = 0;

    if (!MT6701_ReadRegPair(MT6701_REG_ANGLE_MSB, &msb, &lsb))
    {
        // 读取失败，返回上一次成功的值而不是0
        return encoder_raw_cache;
    }

    uint16_t raw = (uint16_t)((msb << 8) | lsb) & 0x3FFF;
    encoder_raw_cache = raw;  // 缓存成功读取的值
    return raw;
}

int16_t Encoder_Get(void)
{
    uint16_t raw = Encoder_GetRaw14();
    int32_t delta = (int32_t)raw - (int32_t)g_last_raw;

    if (delta > (MT6701_ELEC_COUNTS / 2))
    {
        delta -= MT6701_ELEC_COUNTS;
    }
    else if (delta < -(MT6701_ELEC_COUNTS / 2))
    {
        delta += MT6701_ELEC_COUNTS;
    }

    g_accum -= delta;  // 反向编码器
    g_last_raw = raw;
    g_encoder_raw_for_foc = raw;  // 更新FOC用的原始值

    int32_t mod = g_accum % MT6701_MECH_COUNTS;
    if (mod < 0) mod += MT6701_MECH_COUNTS;

    uint16_t angle_tenths = (uint16_t)((uint32_t)mod * 3600U / MT6701_MECH_COUNTS);
    return (int16_t)angle_tenths;
}

int16_t Encoder_GetSpeedRPM(void)
{
    return g_current_rpm;
}

// 编码器角度缓存（在主循环中更新，FOC中使用，避免中断中的I2C阻塞）

// 轴对齐补偿（存储d轴对齐时计算的角度偏移）

// OLED 显示标志位（在FOC中断中设置标志，主循环中显示）

// FOC运行标志位（初始关闭，按键启动）

// FOC状态机：0=正常运行，1=对齐中

/* ==================== FOC Module (Fixed-Point) ==================== */
// 定点数定义 Q15格式：1符号位+15小数位，范围[-1, 0.999969]

// 角度定义：uint16_t 0~65535 对应 0~2π

// FOC参数（根据实际调整）

// 清零FOC积分器（启动时调用）
void FOC_ResetIntegrators(void)
{
    Id_int_q15 = 0;
    Iq_int_q15 = 0;
}

// 启动FOC（从对齐模式开始，对齐完成后自动进入闭环）
void FOC_Start(void)
{
    FOC_ResetIntegrators();
    g_align_counter = 0;
    g_alignment_done = 0;
    g_iq_ref_q15 = 0;  // 默认无力矩，外部设置
    g_foc_state = FOC_STATE_ALIGN;
}

// 停止FOC
void FOC_Stop(void)
{
    g_foc_state = FOC_STATE_IDLE;
    TIM_SetCompare1(TIM2, 0);
    TIM_SetCompare2(TIM2, 0);
    TIM_SetCompare3(TIM2, 0);
}

// 快速Sin查表（带线性插值）
static int16_t fast_sin_q15(uint16_t angle)
{
    uint16_t index = angle >> 8;
    uint16_t frac = angle & 0xFF;
    int16_t s0 = sin_table[index];
    int16_t s1 = sin_table[(index + 1) & 0xFF];
    return s0 + (int16_t)(((int32_t)(s1 - s0) * frac) >> 8);
}

// 快速Cos查表
static int16_t fast_cos_q15(uint16_t angle)
{
    return fast_sin_q15(angle + 16384);
}

// 定点数限幅
static int16_t clamp_q15(int32_t val, int16_t min, int16_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return (int16_t)val;
}



// Clarke变换（两电流版，定点数版本）
// 输入：Q15格式电流，输出：Q15格式
void clarke_transform_q15(int16_t Ia_q15, int16_t Ib_q15, int16_t Ic_q15, int16_t *alpha, int16_t *beta)
{
    (void)Ic_q15;
    *alpha = Ia_q15;

    // Ibeta = (Ia + 2*Ib) / sqrt(3)
    int32_t temp = (int32_t)Ia_q15 + ((int32_t)Ib_q15 << 1);
    *beta = (int16_t)((temp * 18919L) >> 15);  // 18919 = (1/sqrt(3))*32768
}

// 逆Clarke变换（定点数版本）
// 输入：Q15格式alpha/beta，输出：Q15格式三相电流
void inv_clarke_transform_q15(int16_t alpha, int16_t beta, int16_t *Ia, int16_t *Ib, int16_t *Ic)
{
    *Ia = alpha;
    // Ib = -0.5*alpha + 0.866*beta
    int32_t temp_b = (-(int32_t)alpha * 16384L + (int32_t)beta * 28378L) >> 15;
    *Ib = (int16_t)temp_b;
    // Ic = -0.5*alpha - 0.866*beta
    int32_t temp_c = (-(int32_t)alpha * 16384L - (int32_t)beta * 28378L) >> 15;
    *Ic = (int16_t)temp_c;
}

// Park变换（定点数版本）
void park_transform_q15(int16_t alpha, int16_t beta, uint16_t theta, int16_t *d, int16_t *q)
{
    int16_t cos_t = fast_cos_q15(theta);
    int16_t sin_t = fast_sin_q15(theta);
    
    int32_t d_temp = Q15_MULT(alpha, cos_t) + Q15_MULT(beta, sin_t);
    int32_t q_temp = -Q15_MULT(alpha, sin_t) + Q15_MULT(beta, cos_t);
    
    *d = (int16_t)d_temp;
    *q = (int16_t)q_temp;
}

// 逆Park变换（定点数版本）
void inv_park_transform_q15(int16_t vd, int16_t vq, uint16_t theta, int16_t *alpha, int16_t *beta)
{
    int16_t cos_t = fast_cos_q15(theta);
    int16_t sin_t = fast_sin_q15(theta);
    
    int32_t alpha_temp = Q15_MULT(vd, cos_t) - Q15_MULT(vq, sin_t);
    int32_t beta_temp = Q15_MULT(vd, sin_t) + Q15_MULT(vq, cos_t);
    
    *alpha = (int16_t)alpha_temp;
    *beta = (int16_t)beta_temp;
}

// 全局标志：用于虚拟闭环的PI抗饱和
static uint8_t g_pi_saturated_d = 0;
static uint8_t g_pi_saturated_q = 0;

// D轴电流PI控制器（定点数版本，带抗积分饱和）
int16_t pi_id_q15(int16_t err_q15)
{
    int16_t Imax_q15 = (int16_t)(MAX_CURRENT_Q15 >> 0);  // 1.5A限制
    
    // 条件积分：只有当误差同号或输出未饱和时才进行积分（抗Windup）
    // 如果上一拍输出饱和了，就停止积分
    if (!g_pi_saturated_d) {
        Id_int_q15 += Q15_MULT(KI_ID_Q15, err_q15);
        Id_int_q15 = clamp_q15(Id_int_q15, -Imax_q15, Imax_q15);
    } else {
        // 激进的抗Windup：输出已饱和时，清零积分器
        Id_int_q15 = 0;
    }
    
    int32_t output = Q15_MULT(KP_ID_Q15, err_q15) + (Id_int_q15 >> 0);
    int16_t result = clamp_q15(output, -Imax_q15, Imax_q15);
    
    // 记录是否饱和（供下一拍参考）
    g_pi_saturated_d = (output != result) ? 1 : 0;
    return result;
}

// Q轴电流PI控制器（定点数版本，带抗积分饱和）
int16_t pi_iq_q15(int16_t err_q15)
{
    int16_t Imax_q15 = (int16_t)(MAX_CURRENT_Q15 >> 0);  // 1.5A限制
    
    // 条件积分：只有当输出未饱和时才进行积分（抗Windup）
    if (!g_pi_saturated_q) {
        Iq_int_q15 += Q15_MULT(KI_IQ_Q15, err_q15);
        Iq_int_q15 = clamp_q15(Iq_int_q15, -Imax_q15, Imax_q15);
    } else {
        // 激进的抗Windup：输出已饱和时，清零积分器
        Iq_int_q15 = 0;
    }
    
    int32_t output = Q15_MULT(KP_IQ_Q15, err_q15) + (Iq_int_q15 >> 0);
    int16_t result = clamp_q15(output, -Imax_q15, Imax_q15);
    
    // 记录是否饱和（供下一拍参考）
    g_pi_saturated_q = (output != result) ? 1 : 0;
    return result;
}

// SVPWM计算函数（定点数版本）
void svpwm_compute_q15(int16_t Valpha_q15, int16_t Vbeta_q15, uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3)
{
    // 1. αβ → 三相（逆Clarke）
    int32_t Va = Valpha_q15;
    int32_t Vb = -((int32_t)Valpha_q15 >> 1) + Q15_MULT(Vbeta_q15, 28378);  // 28378 = 0.866*32768
    int32_t Vc = -((int32_t)Valpha_q15 >> 1) - Q15_MULT(Vbeta_q15, 28378);
    
    // 2. 零序电压注入
    int32_t Vmax = Va;
    if (Vb > Vmax) Vmax = Vb;
    if (Vc > Vmax) Vmax = Vc;
    
    int32_t Vmin = Va;
    if (Vb < Vmin) Vmin = Vb;
    if (Vc < Vmin) Vmin = Vc;
    
    int32_t Voffset = -(Vmax + Vmin) >> 1;
    
    Va += Voffset;
    Vb += Voffset;
    Vc += Voffset;
    
    // 3. 转换为PWM占空比（考虑母线电压：12V）
    // Q15格式±6V范围 → PWM占空比0~1800
    // 关键修复：电压缩放正确对应母线电压，防止过调制
    uint16_t half_period = PWM_PERIOD_TICKS >> 1;
    int32_t k = (int32_t)half_period;  // 900，PWM周期的一半，对应满幅度
    
    int32_t Ta = half_period + ((Va * k) >> 15);
    int32_t Tb = half_period + ((Vb * k) >> 15);
    int32_t Tc = half_period + ((Vc * k) >> 15);
    
    // 4. 安全限幅
    if (Ta < 0) Ta = 0;
    if (Tb < 0) Tb = 0;
    if (Tc < 0) Tc = 0;
    
    if (Ta > PWM_PERIOD_TICKS) Ta = PWM_PERIOD_TICKS;
    if (Tb > PWM_PERIOD_TICKS) Tb = PWM_PERIOD_TICKS;
    if (Tc > PWM_PERIOD_TICKS) Tc = PWM_PERIOD_TICKS;
    
    *ccr1 = (uint16_t)Ta;
    *ccr2 = (uint16_t)Tb;
    *ccr3 = (uint16_t)Tc;
}

// 传感器数据采集函数：在TIM3中断中调用（10kHz）
// 电流采集已移至TIM2 FOC中断
static void SensorData_Collect(void)
{
    // 电流在TIM2 FOC中断中读取，此处无需重复采集
}

// TIM3中断处理函数（10kHz触发，用于传感器数据采集）
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        // 对ADC采样值进行一阶低通滤波（alpha≈1/16）
        // 这是第一级滤波，在FOC中还有第二级滤波形成双重滤波架构
        static uint8_t adc_filt_init = 0;
        if (!adc_filt_init) {
            g_adc_filt[0] = (int32_t)AD_Value[0] << 4;  // 预加载
            g_adc_filt[1] = (int32_t)AD_Value[1] << 4;
            g_adc_filt[2] = (int32_t)AD_Value[2] << 4;
            adc_filt_init = 1;
        }
        
        g_adc_filt[0] = g_adc_filt[0] + (AD_Value[0] - (g_adc_filt[0] >> 4));
        g_adc_filt[1] = g_adc_filt[1] + (AD_Value[1] - (g_adc_filt[1] >> 4));
        g_adc_filt[2] = g_adc_filt[2] + (AD_Value[2] - (g_adc_filt[2] >> 4));
        
        // 更新公共AD_Value为滤波后的值，供后续处理使用
        AD_Value[0] = (uint16_t)(g_adc_filt[0] >> 4);
        AD_Value[1] = (uint16_t)(g_adc_filt[1] >> 4);
        AD_Value[2] = (uint16_t)(g_adc_filt[2] >> 4);
        
        // 采集传感器数据
        SensorData_Collect();
        
        // 每100次中断设置一次OLED更新标志（100Hz刷新）
        display_counter++;
        if (display_counter >= 100)
        {
            display_counter = 0;
            g_oled_update_flag = 1;  // 通知主循环更新显示
        }
        
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}

// TIM2中断处理函数（最高优先级，FOC控制环 @ 20kHz）
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // 中心对齐模式下更新中断触发频率是PWM频率的2倍(40kHz)
        // 跳过一半，实际FOC频率为20kHz
        static uint8_t foc_skip = 0;
        foc_skip ^= 1;
        if (foc_skip) return;
        
        // 读取三相电流
        // 注：AD_Value已在TIM3中进行了第一级一阶滤波（alpha≈1/16）
        // 这里进行第二级滤波（alpha≈1/256）形成双重滤波，大幅削弱高频噪声
        static int32_t Ia_filt = 0, Ib_filt = 0, Ic_filt = 0;
        static uint8_t filt_init = 0;
        int16_t Ia_raw = (int16_t)DMAAD_AdcToCurrent_mA(AD_Value[0], 0);
        int16_t Ib_raw = (int16_t)DMAAD_AdcToCurrent_mA(AD_Value[1], 1);
        int16_t Ic_raw = (int16_t)DMAAD_AdcToCurrent_mA(AD_Value[2], 2);
        
        // 开环模式使用原始电流（无滤波），闭环模式使用滤波电流
        int16_t Ia_use, Ib_use, Ic_use;
        if (g_foc_state == FOC_STATE_OPENLOOP) {
            // 开环：直接使用原始电流反馈，避免滤波延迟
            Ia_use = Ia_raw;
            Ib_use = Ib_raw;
            Ic_use = Ic_raw;
        } else {
            // 闭环：使用滤波电流
            // 首次运行时预加载滤波器，避免冷启动瞬态
            if (!filt_init) {
                Ia_filt = (int32_t)Ia_raw << 8;
                Ib_filt = (int32_t)Ib_raw << 8;
                Ic_filt = (int32_t)Ic_raw << 8;
                filt_init = 1;
            }
            
            Ia_filt = Ia_filt + (Ia_raw - (Ia_filt >> 8));  // alpha≈1/256
            Ib_filt = Ib_filt + (Ib_raw - (Ib_filt >> 8));
            Ic_filt = Ic_filt + (Ic_raw - (Ic_filt >> 8));
            Ia_use = (int16_t)(Ia_filt >> 8);
            Ib_use = (int16_t)(Ib_filt >> 8);
            Ic_use = (int16_t)(Ic_filt >> 8);
        }
        g_Ia_mA = Ia_use;
        g_Ib_mA = Ib_use;
        g_Ic_mA = Ic_use;
        
        /* ---- 状态机：空闲 ---- */
        if (g_foc_state == FOC_STATE_IDLE)
        {
            TIM_SetCompare1(TIM2, 0);
            TIM_SetCompare2(TIM2, 0);
            TIM_SetCompare3(TIM2, 0);
            return;
        }
        
        /* ---- 状态机：对齐模式 ---- */
        if (g_foc_state == FOC_STATE_ALIGN)
        {
            // 施加d轴电压，将转子锁定到电角度0位置
            int16_t Valpha, Vbeta;
            uint16_t ccr1, ccr2, ccr3;
            
            // Vd=3000(~9%), Vq=0, theta=0 → 锁定到A相
            // 改为较小对齐电压3000(~9%)便于观察虚拟闭环
            inv_park_transform_q15(3000, 0, 0, &Valpha, &Vbeta);
            svpwm_compute_q15(Valpha, Vbeta, &ccr1, &ccr2, &ccr3);
            
            TIM_SetCompare1(TIM2, ccr1);
            TIM_SetCompare2(TIM2, ccr2);
            TIM_SetCompare3(TIM2, ccr3);
            
            g_align_counter++;
            if (g_align_counter >= 20000U)  // 1秒对齐 (20kHz × 1s)
            {
                // 闭环进入时的参数设置
                FOC_ResetIntegrators();  // 清零积分器
                // 设置初始q轴电流参考（较小值，约137mA转矩）
                g_iq_ref_q15 = 3000;
                g_alignment_done = 1;
                g_foc_state = FOC_STATE_CLOSEDLOOP;
            }
            return;
        }
        
        /* ---- 状态机：开环模式 ---- */
        if (g_foc_state == FOC_STATE_OPENLOOP)
        {
            // 软件步进电角度，不依赖电流反馈
            g_open_loop_theta += OPEN_LOOP_STEP_Q16;  // 使用宏定义的步进值 (Q16格式)
            
            int16_t Valpha, Vbeta;
            uint16_t ccr1, ccr2, ccr3;
            
            // Vd=0, Vq=OPEN_LOOP_VQ_Q15 — q轴施加电压驱动转子旋转
            inv_park_transform_q15(0, OPEN_LOOP_VQ_Q15, g_open_loop_theta, &Valpha, &Vbeta);
            svpwm_compute_q15(Valpha, Vbeta, &ccr1, &ccr2, &ccr3);
            
            TIM_SetCompare1(TIM2, ccr1);
            TIM_SetCompare2(TIM2, ccr2);
            TIM_SetCompare3(TIM2, ccr3);
            return;
        }
        
        /* ---- 状态机：闭环FOC模式 ---- */
        if (g_foc_state == FOC_STATE_CLOSEDLOOP)
        {
            // 使用编码器反馈角度
            uint16_t raw14 = g_encoder_raw_for_foc;
            uint32_t elec_angle = (raw14 * MT6701_POLE_PAIRS) % MT6701_ELEC_COUNTS;
            uint16_t theta = (uint16_t)((elec_angle * 65536UL) / MT6701_ELEC_COUNTS);
            
            // 根据g_theta_flip标志决定是否翻转角度方向
            if (g_theta_flip) {
                theta = (uint16_t)(-(int16_t)theta);
            }
            
            // 使用真实三相电流反馈（已滤波，单位：mA）
            // 转换为Q15格式：1.5A = 1500mA 对应 32767
            int16_t Ia_q15 = (int16_t)((int32_t)Ia_use * 32767 / 1500);
            int16_t Ib_q15 = (int16_t)((int32_t)Ib_use * 32767 / 1500);
            int16_t Ic_q15 = (int16_t)((int32_t)Ic_use * 32767 / 1500);
            
            // Clarke变换: Ia,Ib,Ic → Iα,Iβ
            int16_t Ialpha, Ibeta;
            clarke_transform_q15(Ia_q15, Ib_q15, Ic_q15, &Ialpha, &Ibeta);
            
            // Park变换: Iα,Iβ → Id,Iq
            int16_t Id, Iq;
            park_transform_q15(Ialpha, Ibeta, theta, &Id, &Iq);
            
            // 5. PI控制器 (Id_ref=0, Iq_ref=g_iq_ref_q15)
            int16_t Vd = pi_id_q15(0 - Id);
            int16_t Vq = pi_iq_q15(g_iq_ref_q15 - Iq);
            
            // 6. 电压限幅
            Vd = clamp_q15(Vd, -VOLT_MAX_Q15, VOLT_MAX_Q15);
            Vq = clamp_q15(Vq, -VOLT_MAX_Q15, VOLT_MAX_Q15);
            
            // 7. 逆Park变换: Vd,Vq → Vα,Vβ
            int16_t Valpha, Vbeta;
            inv_park_transform_q15(Vd, Vq, theta, &Valpha, &Vbeta);
            
            // 8. SVPWM输出
            uint16_t ccr1, ccr2, ccr3;
            svpwm_compute_q15(Valpha, Vbeta, &ccr1, &ccr2, &ccr3);
            
            TIM_SetCompare1(TIM2, ccr1);
            TIM_SetCompare2(TIM2, ccr2);
            TIM_SetCompare3(TIM2, ccr3);
        }
    }
}

/* ==================== Key Module (PB13) ==================== */

static void Key_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // 下拉输入，按下为高
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 返回1表示按键按下（带消抖），0表示未按下
static uint8_t Key_Pressed(void)
{
    static uint8_t last_state = 0;  // 下拉，默认低
    uint8_t curr = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
    
    if (last_state == 0 && curr == 1)  // 上升沿（按下）
    {
        Delay_ms(20);  // 消抖
        curr = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
        if (curr == 1)
        {
            last_state = 1;
            return 1;  // 确认按下
        }
    }
    else if (curr == 0)
    {
        last_state = 0;  // 松手复位
    }
    return 0;
}

/* ==================== Main ==================== */
int main(void)
{
    // 配置NVIC中断优先级分组（2位抢占，2位子优先级）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    OLED_Init();
    OLED_Clear();
    
    // 立即显示测试信息
    OLED_ShowString(1, 1, "Step 1: OLED OK ");
    Delay_ms(500);
    
    PWM_Init();
    OLED_ShowString(2, 1, "Step 2: PWM OK  ");
    Delay_ms(500);
    
    OLED_ShowString(3, 1, "Step 3: ADC...  ");
    DMAAD_Init();
    OLED_ShowString(3, 1, "Step 3: ADC OK  ");
    Delay_ms(500);
    
    OLED_ShowString(4, 1, "Step 4: ENC...  ");
    Encoder_Init();
    OLED_ShowString(4, 1, "Step 4: ENC OK  ");
    Delay_ms(500);
    
    // 暂时不启动DMA，使用简单的I2C轮询
    // Encoder_StartHighSpeedAcquisition();
    
    OLED_Clear();
    // ADC零点校准（电机静止时运行）
    OLED_ShowString(1, 1, "Calibrating...  ");
    DMAAD_CalibrateZeroPoint();
    
    // 显示初始屏幕
    OLED_ShowString(1, 1, "Ang:   Spd:     ");
    OLED_ShowString(2, 1, "Ia:          mA ");
    OLED_ShowString(3, 1, "Ib:          mA ");
    OLED_ShowString(4, 1, "Ic:          mA ");
    
    // 初始化按键
    Key_Init();
    
    // 启用TIM3中断，开始采集传感器数据
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    Delay_ms(100);
    
    // 默认不启动FOC，等待按键
    // FOC状态已在变量初始化中设为IDLE

    while (1)
    {
        // 按键检测：切换FOC启停
        if (Key_Pressed())
        {
            if (g_foc_state == FOC_STATE_IDLE)
            {
                FOC_Start();  // 启动FOC（对齐→闭环）
            }
            else
            {
                FOC_Stop();   // 停止FOC，PWM归零
            }
        }
        
        // 使用简单的I2C轮询模式（不用DMA）
        int16_t angle_signed = Encoder_Get();
        uint16_t mech_angle_tenths = (uint16_t)angle_signed;
        g_cached_angle_tenths = mech_angle_tenths;
        Encoder_UpdateSpeedFromAccum();
        
        // 只在OLED更新标志设置时才刷新屏幕
        if (g_oled_update_flag)
        {
            g_oled_update_flag = 0;
            
            // 计算显示数据
            uint16_t angle_deg = g_cached_angle_tenths / 10;
            int16_t speed_rpm_signed = Encoder_GetSpeedRPM();
            char speed_sign = speed_rpm_signed < 0 ? '-' : '+';
            uint16_t speed_rpm = (uint16_t)(speed_rpm_signed < 0 ? -speed_rpm_signed : speed_rpm_signed);
            
            // 第1行：角度、速度
            OLED_ShowNum(1, 5, angle_deg, 3);
            OLED_ShowChar(1, 9, speed_sign);
            OLED_ShowNum(1, 11, speed_rpm, 4);
            
            // 第2行：Ia电流
            char ia_sign = g_Ia_mA < 0 ? '-' : '+';
            uint16_t ia_abs = (uint16_t)(g_Ia_mA < 0 ? -g_Ia_mA : g_Ia_mA);
            OLED_ShowChar(2, 4, ia_sign);
            OLED_ShowNum(2, 5, ia_abs, 5);
            
            // 第3行：Ib电流
            char ib_sign = g_Ib_mA < 0 ? '-' : '+';
            uint16_t ib_abs = (uint16_t)(g_Ib_mA < 0 ? -g_Ib_mA : g_Ib_mA);
            OLED_ShowChar(3, 4, ib_sign);
            OLED_ShowNum(3, 5, ib_abs, 5);
            
            // 第4行：Ic电流
            char ic_sign = g_Ic_mA < 0 ? '-' : '+';
            uint16_t ic_abs = (uint16_t)(g_Ic_mA < 0 ? -g_Ic_mA : g_Ic_mA);
            OLED_ShowChar(4, 4, ic_sign);
            OLED_ShowNum(4, 5, ic_abs, 5);
        }
    }
}

