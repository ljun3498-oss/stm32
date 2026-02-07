#include "stm32f10x.h"                  // Device header

#define MT6701_I2C_ADDR        0x06
#define MT6701_REG_ANGLE_MSB   0x03
#define MT6701_ELEC_COUNTS     16384
#define MT6701_POLE_PAIRS      4
#define MT6701_MECH_COUNTS     (MT6701_ELEC_COUNTS * MT6701_POLE_PAIRS)

static volatile uint32_t g_ms = 0;
static uint16_t g_last_raw = 0;
static int32_t g_accum = 0;

static void Encoder_TimeBaseInit(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseInitStructure.TIM_Period = 10 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        g_ms++;
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

static uint8_t MT6701_ReadRegPair(uint8_t reg, uint8_t *msb, uint8_t *lsb)
{
    if (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
    {
        return 0;
    }

    I2C_GenerateSTART(I2C2, ENABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {}

    I2C_Send7bitAddress(I2C2, MT6701_I2C_ADDR << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {}

    I2C_SendData(I2C2, reg);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {}

    I2C_GenerateSTART(I2C2, ENABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {}

    I2C_Send7bitAddress(I2C2, MT6701_I2C_ADDR << 1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {}

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {}
    *msb = I2C_ReceiveData(I2C2);

    I2C_AcknowledgeConfig(I2C2, DISABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {}
    *lsb = I2C_ReceiveData(I2C2);

    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    return 1;
}

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
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    I2C_Init(I2C2, &I2C_InitStructure);

    I2C_Cmd(I2C2, ENABLE);

    g_ms = 0;
    g_last_raw = 0;
    g_accum = 0;
    Encoder_TimeBaseInit();
}

uint16_t Encoder_GetRaw14(void)
{
    uint8_t msb = 0;
    uint8_t lsb = 0;

    if (!MT6701_ReadRegPair(MT6701_REG_ANGLE_MSB, &msb, &lsb))
    {
        return 0;
    }

    return (uint16_t)((msb << 8) | lsb) & 0x3FFF;
}

int16_t Encoder_Get(void)
{
    uint16_t raw = Encoder_GetRaw14();
    int32_t delta = (int32_t)raw - (int32_t)g_last_raw;
    int32_t mod;
    uint16_t angle_tenths;

    if (delta > (MT6701_ELEC_COUNTS / 2))
    {
        delta -= MT6701_ELEC_COUNTS;
    }
    else if (delta < -(MT6701_ELEC_COUNTS / 2))
    {
        delta += MT6701_ELEC_COUNTS;
    }

    g_accum += delta;
    g_last_raw = raw;

    mod = g_accum % MT6701_MECH_COUNTS;
    if (mod < 0) mod += MT6701_MECH_COUNTS;

    angle_tenths = (uint16_t)((uint32_t)mod * 3600U / MT6701_MECH_COUNTS);
    return (int16_t)angle_tenths;
}

int16_t Encoder_GetSpeedRPM(void)
{
    static int32_t last_accum = 0;
    static uint32_t last_ms = 0;
    int32_t delta_counts;
    uint32_t now_ms = g_ms;
    uint32_t dt_ms = now_ms - last_ms;
    int32_t rpm;

    if (dt_ms == 0)
    {
        return 0;
    }

    delta_counts = g_accum - last_accum;
    rpm = (int32_t)((int64_t)delta_counts * 60000LL / (MT6701_MECH_COUNTS * (int32_t)dt_ms));

    last_accum = g_accum;
    last_ms = now_ms;

    if (rpm > 32767) rpm = 32767;
    if (rpm < -32768) rpm = -32768;
    return (int16_t)rpm;
}
