/*
 * button_interrupt.c
 *
 *  Created on: 2021. gada 19. marts
 *      Author: Matiss
 */

#include "stm32l476xx.h"
#include "stdio.h"

int main(void){

	GPIO_Handle_t GpioLed, GpioExtButton, GpioButton;

	//led config
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // with aditional pull up resistor to 3v3 form pin 6
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//external button config
	GpioExtButton.pGPIOx = GPIOA;
	GpioExtButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GpioExtButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioExtButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioExtButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	//built in button config
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioButton);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);


	GPIO_PeriClockControl(GPIOA,ENABLE);


	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioExtButton);


	//IRQ config
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);


	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_7);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
}

void EXTI15_10_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
}

