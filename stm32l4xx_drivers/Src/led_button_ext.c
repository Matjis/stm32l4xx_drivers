/*
 * led_button_ext.c
 *
 *  Created on: 2021. gada 5. marts
 *      Author: Matiss
 */


#include "stm32l476xx.h"
#include "stdio.h"

void delay(void){
	for(uint32_t i=0; i<500000; i++);
}

int main(void){

	GPIO_Handle_t GpioLed1, GpioLed2, GpioExtButton;


	//led 1 config - A6 built in led for L476xx;
	GpioLed1.pGPIOx = GPIOA;
	GpioLed1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//led 2 config
	GpioLed2.pGPIOx = GPIOA;
	GpioLed2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GpioLed2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // with aditional pull up resistor to 3v3 form pin 6
	GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//external button config
	GpioExtButton.pGPIOx = GPIOA;
	GpioExtButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GpioExtButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioExtButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioExtButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA,ENABLE);


	GPIO_Init(&GpioLed1);
	GPIO_Init(&GpioLed2);
	GPIO_Init(&GpioExtButton);


	int state = 0;

	while(1){

		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_7) == 0){
					delay();
					//GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);


					if(state == 0)
						state = 1;
					else
						state = 0;

					GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_6, state);
		}
	}
}
