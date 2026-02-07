#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "Key.H"
#include "OLED.H"
#include "Encoder.H"
#include "Timer.H"
#include "PWM.h"
#include "DMA_ADC.H"

int main(void)
{
	uint16_t duty1 = 0;
	uint16_t duty2 = 0;
	uint16_t duty3 = 0;
	const uint16_t steps = 200;
	uint16_t phase = 0;
	uint16_t p1;
	uint16_t p2;
	uint16_t p3;
	int16_t angle_tenths;
	uint16_t angle_deg;
	uint16_t angle_dec;
	int16_t speed_rpm;

	OLED_Init();		//OLED初始化
	PWM_Init();
	Encoder_Init();
	DMAAD_Init();

	OLED_ShowString(1, 1, "ANG:");
	OLED_ShowString(2, 1, "RPM:");

	while (1)
	{
		angle_tenths = Encoder_Get();
		if (angle_tenths < 0) angle_tenths = 0;
		angle_deg = (uint16_t)(angle_tenths / 10);
		angle_dec = (uint16_t)(angle_tenths % 10);
		speed_rpm = Encoder_GetSpeedRPM();

		p1 = phase;
		p2 = (phase + steps / 3) % steps;
		p3 = (phase + (steps * 2) / 3) % steps;

		if (p1 <= steps / 2) duty1 = (uint16_t)((uint32_t)p1 * PWM_PERIOD_TICKS / (steps / 2));
		else duty1 = (uint16_t)((uint32_t)(steps - p1) * PWM_PERIOD_TICKS / (steps / 2));

		if (p2 <= steps / 2) duty2 = (uint16_t)((uint32_t)p2 * PWM_PERIOD_TICKS / (steps / 2));
		else duty2 = (uint16_t)((uint32_t)(steps - p2) * PWM_PERIOD_TICKS / (steps / 2));

		if (p3 <= steps / 2) duty3 = (uint16_t)((uint32_t)p3 * PWM_PERIOD_TICKS / (steps / 2));
		else duty3 = (uint16_t)((uint32_t)(steps - p3) * PWM_PERIOD_TICKS / (steps / 2));

		PWM_Setcompare1(duty1);
		PWM_Setcompare2(duty2);
		PWM_Setcompare3(duty3);

		OLED_ShowNum(1, 6, angle_deg, 3);
		OLED_ShowChar(1, 9, '.');
		OLED_ShowNum(1, 10, angle_dec, 1);
		OLED_ShowChar(1, 11, 'D');
		OLED_ShowSignedNum(2, 6, speed_rpm, 5);

		phase++;
		if (phase >= steps) phase = 0;

		Delay_ms(20);
	}
}








