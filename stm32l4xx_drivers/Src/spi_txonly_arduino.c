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


void delay(void){
	for(uint32_t i=0; i<250000; i++);
}

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
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // main clock is 16MHz, and we want 2MHz
	SPI2handle.SPIConfig.SPI_CRCL = SPI_CRCN_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave managment for NSS enabled

	SPI_Init(&SPI2handle);
}

void Ext_button_Inits(){
	GPIO_Handle_t GpioExtButton;

	//external button config
	GpioExtButton.pGPIOx = GPIOA;
	GpioExtButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GpioExtButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioExtButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioExtButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GpioExtButton);
}

void User_button_Inits(){
	GPIO_Handle_t GpioUserButton;

	//external button config
	GpioUserButton.pGPIOx = GPIOC;
	GpioUserButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioUserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioUserButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioUserButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GpioUserButton);
}

int main(){

	char user_data[] = "Hello 1 world";

	//this function is needed to set up and initialize SPI2 GPIO pins
	SPI2_GPIOInits();

	//this function is needed to set up and initialize SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * Settin SSOE = 1 makes NSS output enabled.
	 * The NSS pin is automaticli managed by hardware.
	 * SPE = 1 => NSS = 0
	 * SPE = 0 => NSS = 1
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	User_button_Inits();

	while(1){

		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

		delay();

		//enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//first send lenght information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		//to send data
		SPI_SendData(SPI2, (uint8_t* )user_data, strlen(user_data) );

		//confirmes SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

		//disable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}

