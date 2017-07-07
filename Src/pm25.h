#ifndef __SENSAIR_H
#define __SENSAIR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

uint8_t PM25_EnableAutoSend(void);
uint8_t PM25_StopAutoSend(void);
uint8_t PM25_StartMeasurement(void);
uint8_t PM25_StopMeasurement(void);
uint8_t PM25_Read(uint16_t *pm25, uint16_t *pm10);

#endif