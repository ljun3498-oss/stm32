#ifndef _PWM_H
#define _PWM_H

#include "stm32f10x.h"

#define PWM_FREQ_KHZ       2
#define PWM_TIMER_CLK_HZ   72000000
#define PWM_CENTER_ALIGNED 1
#if PWM_CENTER_ALIGNED
#define PWM_PERIOD_TICKS   ((PWM_TIMER_CLK_HZ / (PWM_FREQ_KHZ * 1000U * 2U)) - 1U)
#else
#define PWM_PERIOD_TICKS   ((PWM_TIMER_CLK_HZ / (PWM_FREQ_KHZ * 1000U)) - 1U)
#endif
#define PWM_PERIOD_TOTAL   (PWM_PERIOD_TICKS + 1U)

void PWM_Init(void);
void PWM_Setcompare1(uint16_t Compare);
void PWM_Setcompare2(uint16_t Compare);
void PWM_Setcompare3(uint16_t Compare);


#endif
