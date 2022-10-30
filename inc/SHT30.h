#ifndef SHT30_DRIVER_H
#define SHT30_DRIVER_H

#include "stm32h7XX_hal.h"
#include "stdbool.h"

#define SHT30_I2C_ADDR	 			(0x44 << 1)
#define SHT30_WAIT_TIME_MAX 	100

typedef struct
{
		I2C_HandleTypeDef *i2cHandle;
		bool isAvalible;
	
		float humidity;
		float temperature;
	
}SHT30;

int8_t SHT30_Init(SHT30 *dev, I2C_HandleTypeDef * i2cHandle);
bool SHT30_is_present(SHT30 *dev);
int8_t SHT30_single_shot_measurement(SHT30 *dev);

#endif /* SHT30_DRIVER_H */
