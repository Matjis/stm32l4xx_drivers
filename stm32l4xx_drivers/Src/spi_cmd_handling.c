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

// Command codes
#define COMMAND_LED_CTRL 		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define	LED_OFF					0

// Arduino analog pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

// Arduino LED
#define LED_PIN					9


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

void Ext_LED_Inits(){

	GPIO_Handle_t GpioLed1;

	//led 1 config - A6 built in led for L476xx;
	GpioLed1.pGPIOx = GPIOC;
	GpioLed1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed1);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte){

	if(ackbyte == (uint8_t)0xF5 ){
		//ack
		return 1;
	}

	return 0;
}

int main(){

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	//this function is needed to set up and initialize SPI2 GPIO pins
	SPI2_GPIOInits();

	//this function is needed to set up and initialize SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * Setting SSOE = 1 makes NSS output enabled.
	 * The NSS pin is automatically managed by hardware.
	 * SPE = 1 => NSS = 0
	 * SPE = 0 => NSS = 1
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	//User_button_Inits();
	Ext_button_Inits();
	Ext_LED_Inits();

	while(1){

		while( GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_7) ){
			GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_8, 1);
		}

		GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_8, 0);

		delay();

		//enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

	  // 1. CMD_LED_CTRL

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		// Send command
		SPI_SendData(SPI2, &commandcode, 1);

		// Do dummy read to clear the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write , 1);

		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte) ){

			// Send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args , 2);
		}

		//confirms SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

		//disable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}

