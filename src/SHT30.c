#include "SHT30.h"

static int8_t SHT30_Write(SHT30 *dev, uint8_t *data, uint8_t length);
static int8_t SHT30_Read(SHT30 *dev, uint8_t *data, uint8_t length);

static float SHT30_CalcTemperature(uint16_t rawValue);
static float SHT30_CalcHumidity(uint16_t rawValue);

static void SHT30_Delay(uint32_t msec);


int8_t SHT30_Init(SHT30 *dev, I2C_HandleTypeDef * i2cHandle)
{
	if(dev == NULL || i2cHandle == NULL)
	{
		return -1;
	}
	
	dev->i2cHandle = i2cHandle;
	dev->isAvalible = SHT30_is_present(dev);
	dev->temperature = 0;
	dev->humidity = 0;	
	
	return 0;
}

/**
 * @brief  Check if there is device with adress coresponding to TMP117 adress
 * @param  none
 * @retval 1 if device was found and 0 if dev wasn't found
 */
bool SHT30_is_present(SHT30 *dev)
{
	if(dev == NULL)
	{
		return false;
	}
	const uint32_t	MAX_AMOUNT_OF_IS_READY_TRIALS	= 20;

	if(HAL_I2C_IsDeviceReady(dev->i2cHandle, SHT30_I2C_ADDR, MAX_AMOUNT_OF_IS_READY_TRIALS, SHT30_WAIT_TIME_MAX) == HAL_OK)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief  Triggers measurement and makes readout
 * @param  none
 * @retval 1 if conversion is done and 0 if not
 */
int8_t SHT30_single_shot_measurement(SHT30 *dev)
{
	if(dev == NULL)
	{
		return -1;
	}
	int8_t status = 0x00;
	
	uint8_t temp1[3] = {0x2C, 0x06};
	status = SHT30_Write(dev, &temp1[0], 2);
	if (status != 0)
	{
		return status;
	}
	SHT30_Delay(20);
		
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	status = SHT30_Read(dev, &data[0], 6);
	if (status != 0)
	{
		return status;
	}
	
	uint16_t temperature = (data[0] << 8) | data[1];
	uint16_t humidity = (data[3] << 8) | data[4];

	dev->humidity = SHT30_CalcHumidity(humidity);
	dev->temperature = 	SHT30_CalcTemperature(temperature);
	return status;
}

/**
 * @brief  Perform soft reset
 * @param  none
 * @retval 1 if conversion is done and 0 if not
 */
int8_t SHT30_soft_reset(SHT30 *dev)
{
	if(dev == NULL)
	{
		return -1;
	}
	int8_t status = 0x00;

	uint8_t cmd[3] = {0x30, 0xA2};
	status = SHT30_Write(dev, &cmd[0], 2);
	if (status != 0)
	{
		return status;
	}
	SHT30_Delay(20);
	return status;
}

static float SHT30_CalcTemperature(uint16_t rawValue)
{
  // calculate temperature [?C]
  // T = -45 + 175 * rawValue / (2^16-1)
  return 175.0f * rawValue / 65535.0f - 45.0f;
}

//-----------------------------------------------------------------------------
static float SHT30_CalcHumidity(uint16_t rawValue)
{
  // calculate relative humidity [%RH]
  // RH = rawValue / (2^16-1) * 100
  return 100.0f * rawValue / 65535.0f;
}

static int8_t SHT30_Write(SHT30 *dev, uint8_t *data, uint8_t length)
{
	if(dev == NULL || data == NULL || length <= 0)
	{
		return -1;
	}
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->i2cHandle, SHT30_I2C_ADDR, data, length, SHT30_WAIT_TIME_MAX);
	if(status != HAL_OK)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

static int8_t SHT30_Read(SHT30 *dev, uint8_t *data, uint8_t length)
{
	if(dev == NULL || data == NULL || length <= 0)
	{
		return -1;
	}
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(dev->i2cHandle, SHT30_I2C_ADDR, data, length, SHT30_WAIT_TIME_MAX);
	if(status != HAL_OK)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

/**
 * @brief  Creates time delay
 * @param  msec number of miliseconds to wait
 * @retval none
 */
static void SHT30_Delay(uint32_t msec)
{
	HAL_Delay(msec);
	//osDelay(msec);
}
