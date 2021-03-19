/*
 * stm32l476xx_gpio_driver.h
 *
 *  Created on: 2021. gada 5. janv.
 *      Author: Matiss
 */

#ifndef INC_STM32L476XX_GPIO_DRIVER_H_
#define INC_STM32L476XX_GPIO_DRIVER_H_

#include "stm32l476xx.h"


typedef struct{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			// possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;	// possible values from @GPIO_PIN_PUPD_CONTROL
	uint8_t GPIO_PinOPType;			// possible values from @GPIO_PIN_OPTYPE
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


// This is a Handle structure for a GPIO pin

typedef struct{

	GPIO_RegDef_t *pGPIOx;   			// Pointer to hold the base address of the GPIO port to which pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	// This holds GPIO pin config settings

}GPIO_Handle_t;


// GPIO pin numbers @GPIO_PIN_NUMBER

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


// GPIO pin possible modes @GPIO_PIN_MODES

#define GPIO_MODE_IN 0 //Input mode
#define GPIO_MODE_OUT 1 // Output mode
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4 // Input mode trigger falling edge detection
#define GPIO_MODE_IT_RT 5 // Input mode trigger rising edge detection
#define GPIO_MODE_IT_RFT 6 //Input mode trigger rising / falling edge detection


// GPIO pin possible output types @GPIO_PIN_OPTYPE

#define GPIO_OP_TYPE_PP 0 // push pull
#define GPIO_OP_TYPE_OD 1 // open drain


// GPIO pin possible output speeds @GPIO_PIN_SPEED

#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_HIGH 2
#define GPIO_SPEED_VERY_HIGH 3


// GPIO pin pull up AND pull down config macros @GPIO_PIN_PUPD_CONTROL

#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

								//APIs supported by this driver

//Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


//Init and De-Init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


//IRQ configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32L476XX_GPIO_DRIVER_H_ */
