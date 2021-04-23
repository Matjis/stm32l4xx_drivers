/*
 * stm32l476xx_spi_driver.c
 *
 *  Created on: 2021. gada 19. apr.
 *      Author: Matiss
 */


#include "stm32l476xx_spi_driver.h"


/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}
}


//Init and De-Init
void SPI_Init(SPI_Handle_t *pSPIHandle){

		// Enable peripheral clock

		SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

		// first configure the SPI_CR1 reg

		uint32_t tempreg = 0;

		// 1. configure the device mode
		tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

		// 2. configure the bus config
		if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
			//BIDI mode should be cleared
			tempreg &= ~( 1 << 15);
		}
		else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
			//BIDI mode should be set
			tempreg |= ( 1 << 15);
		}
		else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
			//BIDI mode should be cleared
			tempreg &= ~( 1 << 15);
			//RXONLY bit must be set
			tempreg |= ( 1 << 10);
		}

		// 3. configure the SPI serial clock speed (baud rate)
		tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

		// 4. configure the DFF
		tempreg |= pSPIHandle->SPIConfig.SPI_CRCL << SPI_CR1_CRCL;

		// 5. configure the CPOL
		tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

		// 6. configure the CPHA
		tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

		// all the necessary bits are set in the CR1 register
		pSPIHandle->pSPIx->CR1 = tempreg;
}


void SPI_DeInit(SPI_RegDef_t *pSPIx){

		if(pSPIx == SPI1){
			SPI1_REG_RESET();
		}
		else if(pSPIx == SPI2){
			SPI2_REG_RESET();
		}
		else if(pSPIx == SPI3){
			SPI3_REG_RESET();
		}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}

	return FLAG_RESET;
}

//Data read and write - THIS IS BLOCKING CALL
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len){

		while(Len > 0){

			// 1. wait until TXE is set
			while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

			// 2. check the DFF bit
			if(pSPIx->CR1 & ( 1 << SPI_CR1_CRCL ) ){
				// 16 bit CRCL

				// 1. load the data in to the DR
				pSPIx->DR = *( (uint16_t*) pTXBuffer);

				Len--;
				Len--;

				(uint16_t*) pTXBuffer++;
			}
			else{
				// 8 bit CRCL

				// 1. load the data in to the DR
				pSPIx->DR = *pTXBuffer;

				Len--;

				 pTXBuffer++;
			}
		}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len){

}


/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - This function does IRQ configuration and ISR handling
 *
 * @param[in]         - IRQ number from vector table for specific EXTI line
 * @param[in]         -
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - This function does IRQ Priority configuration
 *
 * @param[in]         - IRQ number from vector table for specific EXTI line
 * @param[in]         -
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

}


void SPI_IRQHandling(SPI_Handle_t *pHandle){

}
