/*
 * spi_tx_testing.c
 *
 *  Created on: 2021. gada 21. apr.
 *      Author: Matiss
 */

#include "stm32l476xx.h"
#include "stdio.h"
#include "string.h"

//SPI2  NSS -> PB9 AF5
//		SCK -> PB10 AF5
//		MISO -> PB14 AF5
//		MOSI -> PB15 AF5


void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// Init SCL - clock pin for SPI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 10;
	GPIO_Init(&SPIPins);

	// Init MISO pin for SPI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&SPIPins);

	// Init MOSI pin for SPI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPIPins);

	// Init NSS pin for SPI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 9;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_CRCL = SPI_CRCN_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // software slave managment for NSS enabled

	SPI_Init(&SPI2handle);
}


int main(){

	char user_data[] = "Hello world";

	//this funcktion is needed to set up and initialize SPI2 GPIO pins
	SPI2_GPIOInits();

	//this funcktion is needed to set up and initialize SPI2 peripheral parameters
	SPI2_Inits();

	//enable SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//to send data
	SPI_SendData(SPI2, (uint8_t* )user_data, strlen(user_data) );

	while(1);

	return 0;
}

