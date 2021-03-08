/*
 * led_toggle.c
 *
 *  Created on: 2021. gada 4. marts
 *      Author: Matiss
 */

#include "stm32l476xx.h"
#include "stdio.h"

void delay(void){
	for(uint32_t i=0; i<25000; i++);
}

int main(void){

	GPIO_Handle_t GpioLed1;
	GPIO_Handle_t GpioLed2;
	//A6 built in led;

	GpioLed1.pGPIOx = GPIOA;
	GpioLed1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioLed2.pGPIOx = GPIOA;
	GpioLed2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GpioLed2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // with aditional pull up resistor to 3v3 form pin 6
	GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed1);
	GPIO_Init(&GpioLed2);


/*
	GPIO_Handle_t GpioLed;
	//PA5 built in led;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);
*/

	while(1){

		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_6, 1);
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		//GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
		delay();

	}
}

