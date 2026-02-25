#include "stm32f10x.h"
#include "delay.h"
#include "OLED.H"
#include <stddef.h>

/* ==================== Global Variables ==================== */

// PWM Module Constants
#define PWM_FREQ_KHZ       5
#define PWM_TIMER_CLK_HZ   72000000
#define PWM_CENTER_ALIGNED 1
#if PWM_CENTER_ALIGNED
#define PWM_PERIOD_TICKS   ((PWM_TIMER_CLK_HZ / (PWM_FREQ_KHZ * 1000U * 2U)) - 1U)
#else
#define PWM_PERIOD_TICKS   ((PWM_TIMER_CLK_HZ / (PWM_FREQ_KHZ * 1000U)) - 1U)
#endif
#define PWM_PERIOD_TOTAL   (PWM_PERIOD_TICKS + 1U)

// Encoder Module Constants
#define MT6701_I2C_ADDR        0x06
#define MT6701_REG_ANGLE_MSB   0x03      // 角度MSB寄存器：存储 Angle[13:6]
#define MT6701_ELEC_COUNTS     16384      // 编码器单圈计数 = 14bit精度
#define MT6701_POLE_PAIRS      7          // 电机极对数：7对
#define MT6701_I2C_TIMEOUT     40       // I2C超时计数（缩短以减少阻塞）
#define THETA_FILT_SHIFT       1          // 电角度一阶滤波系数：1/2

// FOC Module Constants
#define Q15_SCALE       32768
#define Q15_MULT(a, b)  ((int32_t)(a) * (int32_t)(b) >> 15)
#define ANGLE_MAX       65536
#define DEG_TO_ANGLE(deg)  ((uint16_t)((deg) * 65536L / 360))
#define BUS_VOLTAGE_MV  12000        // 母线电压 12V
#define MAX_CURRENT_MA  1500         // 额定电流 1.5A
#define MAX_CURRENT_Q15 32767        // 1.5A对应的Q15值（最大值）
#define KP_ID_Q15       2000         // 降低比例增益
#define KI_ID_Q15       20           // 积分项设置为0
#define KP_IQ_Q15       4000         // 降低比例增益
#define KI_IQ_Q15       20           // 积分项设置为0
#define VOLT_MAX_Q15    26214        // 0.8 in Q15
#define OPEN_LOOP_STEP_Q16 30        // 电角度步进(20kHz更新，极低速度用于测试)
#define CLOSED_LOOP_VQ_TEST_ENABLE 0   // 闭环测试：1=固定Vq绕过电流环
#define CLOSED_LOOP_VQ_TEST_Q15    4000 // 闭环测试时的Vq幅值
#define FORCE_VIRTUAL_THETA        0   // 强制使用虚拟角度拖动
// 电角度偏置(度)
// 范围：0-360，可用于调整电机的初始相位
// 例如：ELEC_ANGLE_OFFSET_DEG = 90   表示电角度偏移90度
//       ELEC_ANGLE_OFFSET_DEG = 180  表示电角度偏移180度
// 转换公式自动处理：度 -> FOC值(0-65535)
#define ELEC_ANGLE_OFFSET_DEG 0
#define ELEC_ANGLE_OFFSET_STEP_DEG 5

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
static uint16_t g_adc_zero_offset[3] = {3098, 3098, 3098};  // 霍尔元件2.5V中点 = 2.5/3.3*4095≈3098
static uint16_t g_adc_filtered[3] = {3098, 3098, 3098};     // 一阶滤波缓存（初始值为零点）

// Encoder Module (non-buffer variables)
static uint16_t g_last_raw = 0;                 // 上一次编码器原始值 [0-16383] - MT6701编码器14bit输出
static int32_t g_accum = 0;                     // 编码器累积值（有符号）- 跨越多圈后的绝对位置，用于角度计算
static int32_t g_last_speed_accum = 0;          // 上一次速度采样的累积值 - 用于计算增量以得到转速
static uint32_t g_last_speed_time = 0;          // 上一次速度采样的时刻（毫秒）- 用于计算时间差
static volatile int16_t g_current_rpm = 0;      // 编码器单圈转速 (RPM)



// 编码器原始值缓存（在读取失败时使用）
static uint16_t encoder_raw_cache = 0;                 // 缓存上一次成功读取的编码器原始值 [0-16383] - I2C读取失败时返回此值
static volatile uint16_t g_encoder_raw_for_foc = 0;    // 供FOC中断用的编码器原始值 [0-16383] - 在Encoder_Get()时更新，FOC中断读取
static volatile uint16_t g_theta_elec = 0;             // FOC用的电角度 [0-65535] - 对应0~2π，= (raw × 极对数) % 65536
static uint16_t g_theta_elec_filt = 0;                 // 电角度滤波后的值 [0-65535]               // 电角度滤波初始化标志

// dq轴电流显示变量
static volatile int16_t g_Id_mA = 0;
static volatile int16_t g_Iq_mA = 0;
static volatile int16_t g_Ia_mA = 0;
static volatile int16_t g_Ib_mA = 0;
static volatile int16_t g_Ic_mA = 0;

// 当前电角度缓存（无论虚拟还是编码器，用于模式切换同步）


// 圈数计数相关全局变量
static int32_t g_encoder_wraparounds = 0;  // 编码器绕过的总次数 - 每次编码器经过0点(0→16384转折)时计数

// FOC Module
#define FOC_STATE_IDLE       0   // 空闲，PWM输出0
#define FOC_STATE_ALIGN      1   // 对齐模式，锁定转子到d轴
#define FOC_STATE_OPENLOOP   2   // 开环模式，验证方向和极对数
#define FOC_STATE_CLOSEDLOOP 3   // 闭环FOC模式

// static uint16_t g_cached_angle_tenths = 0;  // 缓存当前编码器角度（未使用）
static volatile uint8_t g_oled_update_flag = 0;
static volatile uint8_t g_foc_enabled = 0;
static volatile uint8_t g_foc_state = FOC_STATE_IDLE;  // FOC状态机

static volatile uint8_t g_theta_flip = 1;  // 电角度方向翻转（1=反向）- 解决"转一下卡住"
static volatile uint16_t g_elec_angle_offset_deg = ELEC_ANGLE_OFFSET_DEG;  // 运行时可调电角度偏置(度)
static volatile uint32_t g_align_counter = 0;    // 对齐计数器
static volatile uint16_t g_align_theta_reference = 0;  // 对齐完成时记录的机械角度，作为电角度0的参考点
static volatile uint8_t g_align_done = 0;  // 对齐完成标志

static volatile uint8_t g_use_virtual_angle = 0;  // 闭环时是否使用虚拟角度（0=编码器，1=虚拟步进）
static volatile uint16_t g_virtual_theta = 0;     // 虚拟角度（用于调试闭环）
static volatile uint32_t g_i2c_fail_count = 0;    // I2C读取失败计数


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
    
    // 配置TIM2输出触发信号用于ADC同步采样
    // 在PWM中心对齐模式下，每个PWM周期会生成一个TRGO脉冲
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
    
    // 启用TIM2的PWM输出（通用定时器需要此调用）
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    
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
    // TIM3现在仅用于OLED显示计时（1kHz），不用于ADC触发
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = (1000000 / 1000) - 1U;  // 1kHz
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;  // 72MHz / 72 = 1MHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    
    // 配置TIM3中断用于OLED显示更新标志
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
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
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  // 连续转换，保证ADC值持续刷新
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  // 软件触发
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
    // 霍尔元件转换：中点2.5V，灵敏度0.185V/A（185mV/A）
    // Formula: I(mA) = (ADC_value - ADC_zero) * 3300mV/4095 / 185mV*A
    
    int16_t adc_zero = (int16_t)g_adc_zero_offset[channel];  // 零点 ≈ 3098
    int16_t adc_delta = (int16_t)adc - adc_zero;             // 相对于零点的偏移
    
    // 转换：adc_delta转电压(mV)，再除以185mV/A得电流(mA)
    // = (adc_delta * 3300 / 4095) / 0.185
    // = adc_delta * 3300 / (4095 * 0.185)
    // 为避免溢出，先简化：adc_delta * 3300 / 757.575 ≈ adc_delta * 4.35
    return (int32_t)adc_delta * 3300L / 758;  // 约等于除以0.185V/A
}

/* ==================== Encoder Module ==================== */

static void I2C2_BusRecover(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    I2C_Cmd(I2C2, DISABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
    Delay_ms(1);

    for (uint8_t i = 0; i < 9; i++)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_10);
        Delay_ms(1);
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);
        Delay_ms(1);
    }

    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    Delay_ms(1);
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    Delay_ms(1);
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    Delay_ms(1);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// TIM4相关代码已删除，暂时不需要计算转速和测量时间



static uint8_t MT6701_ReadRegPair(uint8_t reg, uint8_t *msb, uint8_t *lsb)
{
    uint32_t timeout;
    
    // 不禁用TIM2中断！MT6701_ReadRegPair在主循环中调用，不会与FOC中断冲突
    // 只需确保I2C总线操作安全，I2C DMA已处理并发
    
    timeout = MT6701_I2C_TIMEOUT;
    // 等待总线空闲（带超时）
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) && timeout > 0) 
    {
        timeout--;
    }
    
    if (timeout == 0) {
        // 总线卡死，软件复位I2C2
        g_i2c_fail_count++;
        I2C2_BusRecover();
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
        return 0;
    }

    I2C_GenerateSTART(I2C2, ENABLE);
    timeout = MT6701_I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) { timeout--; }

    I2C_Send7bitAddress(I2C2, MT6701_I2C_ADDR << 1, I2C_Direction_Transmitter);
    timeout = MT6701_I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout > 0) { timeout--; }

    I2C_SendData(I2C2, reg);
    timeout = MT6701_I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout > 0) { timeout--; }

    I2C_GenerateSTART(I2C2, ENABLE);
    timeout = MT6701_I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && timeout > 0) { timeout--; }

    I2C_Send7bitAddress(I2C2, MT6701_I2C_ADDR << 1, I2C_Direction_Receiver);
    timeout = MT6701_I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout > 0) { timeout--; }

    timeout = MT6701_I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) && timeout > 0) { timeout--; }
    *msb = I2C_ReceiveData(I2C2);

    I2C_AcknowledgeConfig(I2C2, DISABLE);
    timeout = MT6701_I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) && timeout > 0) { timeout--; }
    *lsb = I2C_ReceiveData(I2C2);

    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    // TIM2中断保持启用 - 这里不再禁用/启用
    return 1;
}

// 前向声明，使 Encoder_Init() 能调用
uint16_t Encoder_GetRaw14(void);
void Encoder_Get(void);
int16_t Encoder_GetSpeedRPM(void);
uint16_t Encoder_GetThetaElec(void);

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

    g_last_raw = 0;
    g_accum = 0;
    g_last_speed_accum = 0;
    g_last_speed_time = 0;
    g_current_rpm = 0;
    g_i2c_fail_count = 0;
    g_align_done = 0;  // 初始化对齐完成标志


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





static void Encoder_UpdateThetaFromRaw(uint16_t raw)
{
    // 正确计算电角度：考虑极对数和编码器分辨率
    // 编码器分辨率：14位 = 16384 ticks/圈
    // 电角度范围：0-65535 = 1 tick/电角度
    // 计算方式：电角度 = (raw * 65536 * 极对数) / 16384
    uint32_t theta = (uint32_t)raw * 65536 * MT6701_POLE_PAIRS;
    theta = theta / 16384;
    
    // 归一化到0-65535范围
    theta = theta % 65536;

    if (g_theta_flip)
        theta = 65535 - theta;

    uint16_t theta_offset = DEG_TO_ANGLE(g_elec_angle_offset_deg);
    theta = (theta + theta_offset) % 65536;

    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    g_theta_elec = (uint16_t)theta;
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}



// Encoder_GetTimeMs函数已删除，暂时不需要时间测量

/**
 * 功能：获取编码器原始14bit数值 [0-16383]
 * 返回：编码器原始机械角度值，范围0-16383，对应编码器一圈
 * 说明：
 *   - 通过I2C读取MT6701编码器的角度寄存器 0x03(MSB) 和 0x04(LSB)
 *   - 若I2C读取失败，返回上一次成功的缓存值（容错处理）
 *   - 成功读取时更新encoder_raw_cache，便于失败重试
 *   - 位拼接按MT6701数据手册：
 *     0x03 存储 Angle[13:6]（8位）
 *     0x04 存储 Angle[5:0] 在 bit[7:2]（6位）
 *     公式：raw = (msb << 6) | (lsb >> 2)
 */
uint16_t Encoder_GetRaw14(void)
{
    uint8_t msb = 0;
    uint8_t lsb = 0;

    if (!MT6701_ReadRegPair(MT6701_REG_ANGLE_MSB, &msb, &lsb))
    {
        // 读取失败，返回上一次成功的值而不是0
        return encoder_raw_cache;
    }

    // 按MT6701数据手册的正确位拼接方式
    // msb 是 Angle[13:6]，左移6位得到 bit[13:6]
    // lsb 的 bit[7:2] 是 Angle[5:0]，右移2位得到 bit[5:0]
    uint16_t raw = (uint16_t)(((uint16_t)msb << 6) | (lsb >> 2));
    encoder_raw_cache = raw;  // 缓存成功读取的值
    return raw;
}

/**
 * 功能：更新FOC用的编码器原始值、电角度和多圈计数
 * 说明：
 *   - 读取编码器原始14bit值并更新g_encoder_raw_for_foc供FOC中断使用
 *   - 计算电角度g_theta_elec = (raw × 极对数) % 65536，供FOC使用
 *   - 处理编码器绕过0点的情况
 *   - 累计g_encoder_wraparounds用于电机圈数计算
 *   - 更新g_accum用于速度计算
 */
void Encoder_Get(void)
{
    uint16_t raw = Encoder_GetRaw14();  // 读取编码器 [0-16383]
    int32_t delta = (int32_t)raw - (int32_t)g_last_raw;  // 计算与上次的差值

    // 处理编码器绕过0点的情况：若跳变超过半圈(8192)，反向处理
    if (delta > (MT6701_ELEC_COUNTS / 2))
    {
        delta -= MT6701_ELEC_COUNTS;  // 16384→0的跳变，应理解为-跳变
        g_encoder_wraparounds--;  // 编码器反向绕过一圈
    }
    else if (delta < -(MT6701_ELEC_COUNTS / 2))
    {
        delta += MT6701_ELEC_COUNTS;  // 0→16384的跳变，应理解为+跳变
        g_encoder_wraparounds++;  // 编码器正向绕过一圈
    }

    if (g_theta_flip)
    {
        g_accum += delta;  // 当电角度反转时，累积值也同向变化
    }
    else
    {
        g_accum -= delta;  // 正常情况下的反向计数
    }
    g_last_raw = raw;  // 保存本次读值为下次参考

    Encoder_UpdateThetaFromRaw(raw);

    // 暂时禁用转速计算，因为删除了TIM4
    // g_current_rpm = 0;
}

/**
 * 功能：获取当前编码器转速
 * 返回：转速值 (RPM - 转/分) - 编码器单圈转速
 * 说明：
 *   - 直接返回g_current_rpm全局变量
 *   - 该值由Encoder_ProcessBuffer()定期更新
 *   - 正值=正向旋转，负值=反向旋转
 */
int16_t Encoder_GetSpeedRPM(void)
{
    return g_current_rpm;  // 范围 [-可能的最大转速到+可能的最大转速]
}

/**
 * 功能：获取FOC用的电角度
 * 返回：电角度值 [0-65535]，对应0~2π弧度，已包含极对数
 * 说明：
 *   - 电角度 = (原始编码器值 × 7极对数) % 65536
 *   - 由Encoder_Get()定期更新，确保FOC中断始终获得最新值
 *   - 用于Park/Clarke变换的角度参考
 */
uint16_t Encoder_GetThetaElec(void)
{
    return g_theta_elec;
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
    g_align_theta_reference = 0;  // 重置对齐参考点，使后续计算从原始值开始
    g_virtual_theta = 0;  // 复位虚拟角度
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
    
    // 饱和处理：限制在Q15范围内 [-32768, 32767]
    if (d_temp > 32767) d_temp = 32767;
    if (d_temp < -32768) d_temp = -32768;
    if (q_temp > 32767) q_temp = 32767;
    if (q_temp < -32768) q_temp = -32768;
    
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
    
    // 饱和处理：限制在Q15范围内 [-32768, 32767]
    if (alpha_temp > 32767) alpha_temp = 32767;
    if (alpha_temp < -32768) alpha_temp = -32768;
    if (beta_temp > 32767) beta_temp = 32767;
    if (beta_temp < -32768) beta_temp = -32768;
    
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
    
    // 条件积分：输出饱和时保持积分不变，避免扭矩突然掉零
    if (!g_pi_saturated_d) {
        Id_int_q15 += Q15_MULT(KI_ID_Q15, err_q15);
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
    
    // 条件积分：输出饱和时保持积分不变，避免扭矩突然掉零
    if (!g_pi_saturated_q) {
        Iq_int_q15 += Q15_MULT(KI_IQ_Q15, err_q15);
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


// TIM3中断处理函数（1kHz，用于OLED显示更新）
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        // 每50次中断设置一次OLED更新标志（20Hz刷新，降低CPU负担）
        display_counter++;
        if (display_counter >= 50)
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
    // 必须清除中断标志，否则会立即再次触发导致卡死
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    
    // ============ FOC 主控制循环 ============
    // 运行频率：由TIM2中断频率决定（通常为电流采样时刻）
    
    if (!g_foc_enabled || g_foc_state == FOC_STATE_IDLE)
    {
        // FOC未启用或空闲状态，输出0
        TIM_SetCompare1(TIM2, 0);
        TIM_SetCompare2(TIM2, 0);
        TIM_SetCompare3(TIM2, 0);
        return;
    }
    
    // 1. 更新虚拟角度（在中断中执行，获得平稳的更新频率）
    if (g_use_virtual_angle)
    {
        g_virtual_theta = (uint16_t)(g_virtual_theta + OPEN_LOOP_STEP_Q16);
    }
    
    // 2. 获取当前电角度，并减去对齐偏置
    uint16_t theta;
    if (g_use_virtual_angle)
    {
        theta = g_virtual_theta;  // 虚拟角度不需要偏置
    }
    else
    {
        // 闭环模式下，所有电角度都要减去对齐时的偏置
        if (g_align_done && g_foc_state == FOC_STATE_CLOSEDLOOP)
        {
            theta = (uint16_t)((int32_t)g_theta_elec - (int32_t)g_align_theta_reference);
        }
        else
        {
            theta = g_theta_elec;  // 0-65535范围，已包含极对数
        }
    }
    
    // 2. 对电流进行一阶低通滤波，降低噪声
    // 滤波系数α = 0.25，公式：y_filtered = (x_raw + 3 * y_old) / 4
    // 这样可以平滑ADC采样的高频噪声，同时保持相对快的响应速度
    for (int i = 0; i < 3; i++)
    {
        g_adc_filtered[i] = (uint16_t)(((uint32_t)AD_Value[i] + (uint32_t)g_adc_filtered[i] * 3) >> 2);
    }
    
    // 3. 读取三相电流（从滤波后的ADC采样值转换）
    // g_adc_filtered[0-2] 是滤波后的12bit值 (0-4095)
    // 使用 DMAAD_AdcToCurrent_mA() 转换为mA
    // 再转换为Q15格式：1500mA对应Q15的32767
    
    // 反转电流反馈符号，用于验证电流极性是否相反
    int32_t Ia_mA = -DMAAD_AdcToCurrent_mA(g_adc_filtered[0], 0);  // 通道0 PA5
    int32_t Ib_mA = -DMAAD_AdcToCurrent_mA(g_adc_filtered[1], 1);  // 通道1 PA6
    int32_t Ic_mA = -DMAAD_AdcToCurrent_mA(g_adc_filtered[2], 2);  // 通道2 PA7

    // 缓存相电流用于OLED显示（主循环读取，避免阻塞FOC）
    g_Ia_mA = (int16_t)Ia_mA;
    g_Ib_mA = (int16_t)Ib_mA;
    g_Ic_mA = (int16_t)Ic_mA;
    
    // 转换为Q15格式 (Q15_MAX = 32767对应1500mA)
    int16_t Ia_q15 = (int16_t)((Ia_mA * 32767L) / 1500);
    int16_t Ib_q15 = (int16_t)((Ib_mA * 32767L) / 1500);
    int16_t Ic_q15 = (int16_t)((Ic_mA * 32767L) / 1500);
    
    // 4. Clarke变换：三相 → αβ
    int16_t alpha_q15, beta_q15;
    clarke_transform_q15(Ia_q15, Ib_q15, Ic_q15, &alpha_q15, &beta_q15);
    
    // 5. Park变换：αβ → dq（使用当前电角度）
    int16_t Id_q15, Iq_q15;
    park_transform_q15(alpha_q15, beta_q15, theta, &Id_q15, &Iq_q15);
    
    // 缓存d-q轴电流用于显示
    g_Id_mA = (Id_q15 * 100) >> 15;  // 转换为mA用于显示
    g_Iq_mA = (Iq_q15 * 100) >> 15;
    
    // 6. PI控制器
    int16_t Vd_q15 = 0, Vq_q15 = 0;
    
    if (g_foc_state == FOC_STATE_ALIGN)
    {
        // ===== 对齐模式：固定d轴电压，q轴为0 =====
        // 目的：将转子锁到d轴（磁通轴），在此位置建立同步，记录该位置为电角度=0的参考
        
        g_align_counter++;
        
        if (g_align_counter < 15000)  // 对齐持续1.5秒（TIM2中断频率20kHz时：15000/20000=0.75s实际时间，需调整为1500ms标准）
        {
            // 对齐时直接给Vq到SVPWM，不经过反Park变换
            // 这里使用固定的αβ电压，相当于给d轴励磁
            int16_t Valpha_q15 = 12000;
            int16_t Vbeta_q15 = 0;
            
            // SVPWM计算输出
            uint16_t ccr1, ccr2, ccr3;
            svpwm_compute_q15(Valpha_q15, Vbeta_q15, &ccr1, &ccr2, &ccr3);
            
            // 更新PWM占空比（实际交换B/C相）
            TIM_SetCompare1(TIM2, ccr1);
            TIM_SetCompare2(TIM2, ccr2);  // 交换B/C相
            TIM_SetCompare3(TIM2, ccr3);  // 交换B/C相
            
            return;  // 直接返回，跳过后续的变换和计算
        }
        else
            {
                // ===== 对齐完成：记录当前电角度为电角度=0的参考点 =====
                // 此时转子已锁定到d轴，当前位置就是电角度全周期的起点
                // 之后的电角度计算会相对于这个点进行
                g_align_theta_reference = g_theta_elec;  // 记录对齐时的电角度
                g_align_done = 1;  // 设置对齐完成标志
                
                g_foc_state = FOC_STATE_CLOSEDLOOP;  // 转入闭环模式
                FOC_ResetIntegrators();
            }
    }
    else if (g_foc_state == FOC_STATE_CLOSEDLOOP)
    {
        // ===== 闭环FOC模式：使用编码器反馈控制 =====
    #if CLOSED_LOOP_VQ_TEST_ENABLE
        // 闭环测试：固定Vq验证电角度方向与SVPWM
        Vd_q15 = 0;
        Vq_q15 = CLOSED_LOOP_VQ_TEST_Q15;
    #else
        // d轴参考值：应该接近0（只用于励磁）
        int16_t Id_ref_q15 = 0;   // 参考d电流（待调试）
        int16_t Iq_ref_q15 = 6000;  // 参考q电流（转矩，提高目标值）
        
        // PI控制：根据误差调节电压
        int16_t Id_err = Id_ref_q15 - Id_q15;
        int16_t Iq_err = Iq_ref_q15 - Iq_q15;
        
        Vd_q15 = pi_id_q15(Id_err);
        Vq_q15 = pi_iq_q15(Iq_err);
    #endif
    }
    
    // 7. 逆Park变换：dq → αβ（仅在闭环模式下执行）
    int16_t Valpha_q15, Vbeta_q15;
    inv_park_transform_q15(Vd_q15, Vq_q15, theta, &Valpha_q15, &Vbeta_q15);
    
    // 8. SVPWM计算输出
    uint16_t ccr1, ccr2, ccr3;
    svpwm_compute_q15(Valpha_q15, Vbeta_q15, &ccr1, &ccr2, &ccr3);
    
    // 9. 更新PWM占空比（实际交换B/C相）
    TIM_SetCompare1(TIM2, ccr1);
    TIM_SetCompare2(TIM2, ccr2);  // 交换B/C相
    TIM_SetCompare3(TIM2, ccr3);  // 交换B/C相
}

/* ==================== Key Module (PB13 & PB14) ==================== */

static void Key_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // 下拉输入，按下为高
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;  // PB13: 启停，PB14: 切换虚拟角度
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 返回1表示按键按下（带消抖），0表示未按下
// key_pin: 要检测的GPIO引脚（GPIO_Pin_13 或 GPIO_Pin_14）
static uint8_t Key_IsPressed(uint16_t key_pin)
{
    static uint8_t last_state_13 = 0;  // PB13状态
    static uint8_t last_state_14 = 0;  // PB14状态
    uint8_t *last_state = (key_pin == GPIO_Pin_13) ? &last_state_13 : &last_state_14;
    
    uint8_t curr = GPIO_ReadInputDataBit(GPIOB, key_pin);
    
    if (*last_state == 0 && curr == 1)  // 上升沿（按下）
    {
        Delay_ms(20);  // 消抖
        curr = GPIO_ReadInputDataBit(GPIOB, key_pin);
        if (curr == 1)
        {
            *last_state = 1;
            return 1;  // 确认按下
        }
    }
    else if (curr == 0)
    {
        *last_state = 0;  // 松手复位
    }
    return 0;
}

// 检查PB13是否按下（设置编码器偏置）
static uint8_t Key_PB13_Pressed(void)
{
    return Key_IsPressed(GPIO_Pin_13);
}

// 检查PB14是否按下（切换显示页）
static uint8_t Key_PB14_Pressed(void)
{
    return Key_IsPressed(GPIO_Pin_14);
}

/* ==================== DMA Mode Select (PA8) ==================== */



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
    
    
    OLED_Clear();
    // ADC零点校准（电机静止时运行）
    OLED_ShowString(1, 1, "Step 5: CAL...  ");
    DMAAD_CalibrateZeroPoint();
    OLED_ShowString(1, 1, "Step 5: CAL OK  ");
    Delay_ms(500);

    // 初始化按键
    OLED_ShowString(2, 1, "Step 6: KEY...  ");
    Key_Init();
    OLED_ShowString(2, 1, "Step 6: KEY OK  ");
    Delay_ms(500);

    // 确保I2C总线状态正常
    OLED_ShowString(3, 1, "Step 7: I2C...  ");
    I2C_GenerateSTOP(I2C2, ENABLE);
    OLED_ShowString(3, 1, "Step 7: I2C OK  ");
    Delay_ms(500);

    // 上电先对齐，再进入闭环FOC
    OLED_ShowString(4, 1, "Step 8: FOC...  ");
    g_foc_enabled = 1;
    FOC_Start();
    OLED_ShowString(4, 1, "Step 8: FOC OK  ");
    Delay_ms(500);
    OLED_Clear();
    // 启用TIM3中断，开始采集传感器数据
    OLED_ShowString(1, 1, "Step 9: TIM...  ");
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    OLED_ShowString(1, 1, "Step 9: TIM OK  ");
    Delay_ms(100);
    
    OLED_Clear();
    OLED_ShowString(1, 1, "Init Complete  ");
    OLED_ShowString(2, 1, "PB13: Start    ");
    OLED_ShowString(3, 1, "PB14: Mode     ");
    OLED_ShowString(4, 1, "                ");
    Delay_ms(1000);
    
    
    while (1)
    {

 
        // 更新编码器角度（在主循环中执行，减少中断开销）
        if (!g_use_virtual_angle)
        {
            uint16_t raw = Encoder_GetRaw14();
            // 处理编码器数据并更新电角度
            Encoder_UpdateThetaFromRaw(raw);
        }
        // 虚拟角度更新现在在TIM2中断中执行，以获得平稳的更新频率
        
        // OLED显示（低频刷新，减少对FOC的影响）
        if (g_oled_update_flag)
        {
            g_oled_update_flag = 0;

            // 正常显示模式
            // 第一行显示虚拟角度
            OLED_ShowString(1, 1, "VT:");
            OLED_ShowNum(1, 4, g_virtual_theta, 5);

            // 第二行显示真实角度（编码器电角度）
            OLED_ShowString(2, 1, "RT:");
            OLED_ShowNum(2, 4, g_theta_elec, 5);

            // 第三行显示d轴电流
            OLED_ShowString(3, 1, "Id:");
            OLED_ShowSignedNum(3, 4, g_Id_mA, 4);

            // 第四行显示q轴电流
            OLED_ShowString(4, 1, "Iq:");
            OLED_ShowSignedNum(4, 4, g_Iq_mA, 4);
        }
        
        // ========== 按键处理 ==========
        
        // PB13: 循环切换FOC状态：IDLE → ALIGN → CLOSEDLOOP → IDLE
        if (Key_PB13_Pressed())
        {
            if (g_foc_state == FOC_STATE_IDLE)
            {
                // 从IDLE切换到ALIGN
                g_foc_enabled = 1;
                FOC_Start();  // 进入对齐模式
            }
            else if (g_foc_state == FOC_STATE_ALIGN)
            {
                // 从ALIGN切换到CLOSEDLOOP
                // 通过设置计数器跳过对齐，直接进入闭环
                g_align_counter = 15000;  // 触发对齐完成条件（与TIM2中的阈值保持一致）
            }
            else if (g_foc_state == FOC_STATE_CLOSEDLOOP)
            {
                // 从CLOSEDLOOP切换到IDLE
                g_foc_enabled = 0;
                FOC_Stop();   // 停止FOC
                g_foc_state = FOC_STATE_IDLE;  // 确保设置为IDLE
            }
            
            Delay_ms(500);  // 按键防抖延迟
        }

        // PB14: 切换虚拟角度和编码器角度
        if (Key_PB14_Pressed())
        {
            g_use_virtual_angle = !g_use_virtual_angle;
            g_oled_update_flag = 1;
            Delay_ms(200);
        }
    }
}

