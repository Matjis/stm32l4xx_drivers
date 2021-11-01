/*
 * stm32l476xx_spi_driver.c
 *
 *  Created on: 2021. gada 19. apr.
 *      Author: Matiss
 */


#include "stm32l476xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

		// 4. configure the CRCL (DFF)
		tempreg |= pSPIHandle->SPIConfig.SPI_CRCL << SPI_CR1_CRCL;

		// 5. configure the CPOL
		tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

		// 6. configure the CPHA
		tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

		tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

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

				/* 1. load the data in to the DR.
				   pSPIx->DR needs to be casted to 8 bit size, otherwise it sends 16 bits.
				*/
				*((uint8_t*) &pSPIx->DR) = *pTXBuffer;
				//*((uint8_t*) &pSPIx->DR) = *((uint8_t*)pTXBuffer); // alternative option

				/* 2. because 8 bits are used FIFO reception threshold needs to be changed to
					1/4 as explained in SPI CR2 register for FRXTH bit. If this isn't done
					SPI_GetFlagStatus() function goes into loop, because of 16 bit threshold.
				*/
				pSPIx->CR2 |= ( 1 << 12 );

				Len--;

				 pTXBuffer++;
			}
		}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len){

	while(Len > 0){

		// 1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. check the DFF bit
		if(pSPIx->CR1 & ( 1 << SPI_CR1_CRCL ) ){
			// 16 bit CRCL

			// 1. load the data from DR to RXbuffer address
			 *( (uint16_t*) pRXBuffer ) = pSPIx->DR;

			Len--;
			Len--;
			(uint16_t*) pRXBuffer++;
		}
		else{
			// 8 bit CRCL

			// 1. load the data in to the DR
			*(pRXBuffer) = pSPIx->DR ;

			Len--;
			pRXBuffer++;
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             - This function does IRQ configuration and ISR handling
 *
 * @param[in]         - IRQ number from vector table for specific SPI line
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if (IRQNumber <= 31){
			//program ISER0 reg
			*NVIC_ISER0	|= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64){ // 32 to 64
			//program ISER1 reg
			*NVIC_ISER1	|= (1 << IRQNumber % 32);
		}
		else if (IRQNumber > 64 && IRQNumber < 96){
			//program ISER2 reg
			*NVIC_ISER2	|= (1 << IRQNumber % 64);
		}
	}
	else{
		if (IRQNumber <= 31){
			//program ISER0 reg
			*NVIC_ICER0	|= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64){ // 32 to 64
			//program ISER1 reg
			*NVIC_ICER1	|= (1 << IRQNumber % 32);
		}
		else if (IRQNumber > 64 && IRQNumber < 96){
			//program ISER2 reg
			*NVIC_ICER2	|= (1 << IRQNumber % 64);
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This function does IRQ Priority configuration
 *
 * @param[in]         - IRQ number from vector table for specific SPI line
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */



void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	//1. find out ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 *iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pSPIx->CR1  |= (1 << SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1  &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pSPIx->CR1  |= (1 << SPI_CR1_SSI);
	}
	else{
		pSPIx->CR1  &= ~(1 << SPI_CR1_SSI);
	}


}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pSPIx->CR2  |= (1 << SPI_CR2_SSOE);
	}
	else{
		pSPIx->CR2  &= ~(1 << SPI_CR2_SSOE);
	}


}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}

	return FLAG_RESET;
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->TXState;

	if( state != SPI_BUSY_IN_TX){
		// 1
		pSPIHandle->pTXBuffer = pTXBuffer;
		pSPIHandle->TXLen = Len;

		// 2
		pSPIHandle->TXState = SPI_BUSY_IN_TX;

		// 3
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	}
	// 4

	return state;

}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->RXState;

	if( state != SPI_BUSY_IN_RX){
		// 1
		pSPIHandle->pRXBuffer = pRXBuffer;
		pSPIHandle->RXLen = Len;

		// 2
		pSPIHandle->RXState = SPI_BUSY_IN_RX;

		// 3
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}
	// 4

	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle){

	uint8_t temp1, temp2;
	// Check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE ); // check if SR 1 possiton to the left is same with 1
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE );

	if ( temp1 && temp2){

		// Handle_TXE
		spi_txe_interrupt_handle();
	}

	// Check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE ); // check if SR 1 possiton to the left is same with 0
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE );

	if ( temp1 && temp2){

		// Handle_RXNE
		spi_rxne_interrupt_handle();
	}

	// Check for OVR flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR ); // check if SR 1 possiton to the left is same with 0
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE );

	if ( temp1 && temp2){

		// Handle_RXNE
		spi_ovr_err_interrupt_handle();
	}

}

// Some helper function implemantion

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	// check the DFF bit
		if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_CRCL ) ){
			// 16 bit CRCL

			// 1. load the data in to the DR
			pSPIHandle->pSPIx->DR = *( (uint16_t*) pSPIHandle->pTXBuffer);

			pSPIHandle->TXLen--;
			pSPIHandle->TXLen--;

			(uint16_t*) pSPIHandle->pTXBuffer++;
		}
		else{
			// 8 bit CRCL

			*((uint8_t*) &pSPIHandle->pSPIx->DR) = pSPIHandle->pTXBuffer;
			pSPIHandle->pSPIx->CR2 |= ( 1 << 12 );
			pSPIHandle->TXLen--;
			pSPIHandle->pTXBuffer++;
		}

		if( !pSPIHandle->TXLen ){

			pSPIHandle->pSPIx->CR2 &= ~ ( 1 << SPI_CR2_TXEIE);
			pSPIHandle->pTXBuffer = NULL;
			pSPIHandle->TXLen = 0;
			pSPIHandle->TXState = SPI_READY;
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}

}
//static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){

	// check the DFF bit
		if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_CRCL ) ){
			// 16 bit CRCL

			// 1. load the data in to the DR from RXbuffer address
			*( (uint16_t*) pSPIHandle->pTXBuffer) = pSPIHandle->pSPIx->DR;

			pSPIHandle->RXLen--;
			pSPIHandle->RXLen--;

			(uint16_t*) pSPIHandle->pRXBuffer++;
		}
		else{
			// 8 bit CRCL

			pSPIHandle->pRXBuffer = pSPIHandle->pSPIx->DR;
			pSPIHandle->RXLen--;
			pSPIHandle->pRXBuffer++;
		}

		if( !pSPIHandle->RXLen ){

			pSPIHandle->pSPIx->CR2 &= ~ ( 1 << SPI_CR2_RXNEIE);
			pSPIHandle->pRXBuffer = NULL;
			pSPIHandle->RXLen = 0;
			pSPIHandle->RXState = SPI_READY;
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}
}

//static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
