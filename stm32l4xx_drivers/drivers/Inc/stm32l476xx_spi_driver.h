/*
 * stm32l476xx_spi_driver.h
 *
 *  Created on: 2021. gada 19. apr.
 *      Author: Matiss
 */

#ifndef INC_STM32L476XX_SPI_DRIVER_H_
#define INC_STM32L476XX_SPI_DRIVER_H_

#include "stm32l476xx.h"


// Configuration structure for SPIx peripheral

typedef struct{

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_CRCL;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;


// This is a Handle structure for a SPIx peripheral

typedef struct{

	SPI_RegDef_t *pSPIx;   		// Pointer to hold the base address of the SPIx (x: 0,1,2) port to which pin it belongs
	SPI_Config_t SPIConfig;		// This holds SPI pin config settings

}SPI_Handle_t;


// @SPI_DeviceMode

#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0


// @SPI_BusConfig

#define SPI_BUS_CONFIG_FD				1  // full duplex
#define SPI_BUS_CONFIG_HD				2  // half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3


// @SPI_SclkSpeed

#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7


// @SPI_DFF

#define SPI_CRCN_8BITS	0
#define SPI_CRCN_16BITS	1


// @SPI_CPOL

#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0  //default


// @SPI_CPHA

#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0  //default


//	@SPI_SSM

#define SPI_SSM_EN	1
#define SPI_SSM_DI	0  //default

//  SPI related status flag definitions

#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)


							//APIs supported by this driver

//Peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


//Init and De-Init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


//Data read and write
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len);


//IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

// Other peripheral control APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

#endif /* INC_STM32L476XX_SPI_DRIVER_H_ */
