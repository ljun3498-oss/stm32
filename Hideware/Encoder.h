#ifndef _ENCODER_H
#define _ENCODER_H

#include "stm32f10x.h"

// Returns mechanical angle in 0.1 degree units (0..3600), pole-pair compensated.
int16_t Encoder_Get(void);
uint16_t Encoder_GetRaw14(void);
// Returns signed speed in RPM.
int16_t Encoder_GetSpeedRPM(void);
void Encoder_Init(void);






#endif
