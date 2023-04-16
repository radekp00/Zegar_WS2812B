/*
 * stm_dht22.h
 *
 *  Created on: 07.07.2018
 *      Author: Wojtek
 */

#ifndef STM_DHT22_H_
#define STM_DHT22_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/*
 * Example USE:
 * uint8_t rawReadedData[5];
 * Dht22_ReadedData_T DHTReadData;
 *
 * Dht22_Initial();
 *
 * while(1)
 * {
 * 		if(Dht22_ReadData(rawReadedData, sizeof(rawReadedData))
		{
			DHTReadData.dht22_Temp = Dht22_calculateTemperature(rawReadedData[1], rawReadedData[2]);
			DHTReadData.dht22_Hum = Dht22_calculateHumidity(rawReadedData[3], rawReadedData[4]);
		}
 * }
 *
 */
#define GPIO_DHT_PORT			GPIOA
#define GPIO_DHT_PIN			GPIO_PIN_13
#define GPIO_DHT_CLOCK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()

typedef struct Dht22_ReadedData_Typedef{
	float dht22_Temp;
	float dht22_Hum;
}Dht22_ReadedData_T;

/* Initialization */
void Dht22_Initial(void);
uint8_t DWT_COUNTER_ENABLE(void);
void delayTime(uint32_t us);
/* Read data */
uint8_t Dht22_ReadData(uint8_t *data, uint8_t tableSize);

uint8_t Dht22_CheckSumControl(uint8_t *data);

/* Calculate Data */
float Dht22_calculateTemperature(uint8_t rawRecTemp_1, uint8_t rawRecTemp_2);
float Dht22_calculateHumidity(uint8_t rawRecHum_1, uint8_t rawRecHum_2);

#endif /* STM_DHT22_H_ */
