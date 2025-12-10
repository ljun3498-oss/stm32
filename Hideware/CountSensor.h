#ifndef _COUNT_SENSOR_H
#define _COUNT_SENSOR_H
void CountSensor_Init(void);
uint16_t CountSensor_Get(void);
void  RTCAlarm_IRQHandler (void);
#endif
