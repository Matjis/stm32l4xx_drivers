/*
 * stm32l476xx_gpio_driver.c
 *
 *  Created on: 2021. gada 5. janv.
 *      Author: Matiss
 */


#include "stm32l476xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}
}


//Init and De-Init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp=0; // temp register

	// 1) configure the mode of input mode

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // multiplication by 2 for pin number means that mode register is 2 bit in size
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); // clearing register
		pGPIOHandle->pGPIOx->MODER |= temp; // setting register
	}
	else{

	}

	temp = 0;

	// 2) configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // multiplication by 2 for pin number means that mode register is 2 bit in size
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); // clearing register
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // setting register

	temp = 0;

	// 3) configure the pupd settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // multiplication by 2 for pin number means that mode register is 2 bit in size
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); // clearing register
	pGPIOHandle->pGPIOx->PUPDR |= temp; // setting register

	temp = 0;

	// 4) configure the out put type - optype

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // multiplication by 2 for pin number means that mode register is 2 bit in size
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing register
	pGPIOHandle->pGPIOx->OTYPER |= temp; // setting register

	temp = 0;

	// 5) configure the alt functionality

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
			pGPIOHandle->pGPIOx->AFRL &= ~( 0xF << (4 * temp)); // clearing register
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp) ); // setting register
		}
		else{
			pGPIOHandle->pGPIOx->AFRH &= ~( 0xF << (4 * temp)); // clearing register
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp) ); // setting register
		}

		temp = 0;
	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	/*
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}*/
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads value from GPIO pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pinNumber macros
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)( (pGPIOx->IDR >>PinNumber) & 0x00000001);

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads value from GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;

	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes value to GPIO pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pinNumber macros
 * @param[in]         -
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes value to GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}


/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles GPIO pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pinNumber macros
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}


//IRQ configuration and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
