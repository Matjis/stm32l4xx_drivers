/*
 * stm32l476xx_i2c_driver.h
 *
 *  Created on: 2022. gada 3. janv.
 *      Author: Matiss
 */

#ifndef INC_STM32L476XX_I2C_DRIVER_H_
#define INC_STM32L476XX_I2C_DRIVER_H_

#include "stm32l476xx.h"


// Configuration structure for I2Cx peripheral

typedef struct{

	uint8_t 	SPI_DeviceMode;
	uint8_t 	SPI_BusConfig;
	uint8_t 	SPI_SclkSpeed;
	uint8_t 	SPI_CRCL;
	uint8_t 	SPI_CPOL;
	uint8_t 	SPI_CPHA;
	uint8_t 	SPI_SSM;

}I2C_Config_t;

#endif /* INC_STM32L476XX_I2C_DRIVER_H_ */
