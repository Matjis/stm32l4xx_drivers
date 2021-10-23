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

	// Enable peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1) configure the input mode

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // multiplication by 2 for pin number means that mode register is 2 bit in size
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); // clearing register
		pGPIOHandle->pGPIOx->MODER |= temp; // setting register
		temp = 0;
	}
	else{
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); // clearing register
		//pGPIOHandle->pGPIOx->MODER |= ( 0x0 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ); // setting register


		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 32;

		//interupt config part

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 32){
				//1. configure the FTSR - falling trigger selection register
				EXTI->FTSR1 |= (1 << temp);

				//clear the corresponding RTSR bit
				EXTI->RTSR1 &= ~(1 << temp);

				temp = 0;
			}
			else{
				//1. configure the FTSR - falling trigger selection register
				EXTI->FTSR2 |= (1 << temp);

				//clear the corresponding RTSR bit
				EXTI->RTSR2 &= ~(1 << temp);

				temp = 0;
			}
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 32){
				//1. configure the RTSR - rising trigger selection register
				EXTI->RTSR1 |= (1 << temp);

				//clear the corresponding RTSR bit
				EXTI->FTSR1 &= ~(1 << temp);

				temp = 0;
			}
			else{
				//1. configure the RTSR - rising trigger selection register
				EXTI->RTSR2 |= (1 << temp);

				//clear the corresponding RTSR bit
				EXTI->FTSR2 &= ~(1 << temp);

				temp = 0;
			}
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 32){
				//1. configure both FTSR and RTSR - falling and rising trigger selection register
				EXTI->FTSR1 |= (1 << temp);
				EXTI->RTSR1 |= (1 << temp);

				temp = 0;
			}
			else{
				//1. configure both FTSR and RTSR - falling and rising trigger selection register
				EXTI->FTSR2 |= (1 << temp);
				EXTI->RTSR2 |= (1 << temp);

				temp = 0;

			}
		}


		 //2. configure the GPIO port selection in SYSCFG_EXTICR

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portCode << ( temp2 * 4);


		//3. enable the exti interrupt delivery using IMR - interrupt mask register

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %32;

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 32){
			EXTI->IMR1 |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		}
		else{
			EXTI->IMR2 |= 1 << temp;
		}

		temp = 0;
	}

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

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
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
	}
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
 * @param[in]         - base address of the gpio peripheral - GPIOx (x = A-F)
 * @param[in]         - pinNumber macros - GPIO_PIN_NO_x (x = 0-15)
 * @param[in]         - output value (1 or 0)
 *
 * @return            -  void
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

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	//1. find out ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 *iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

}

void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the EXTI PR register corresponding to the pin number
	uint32_t temp = PinNumber % 32;

	if(PinNumber < 32){
		if(EXTI->PR1 & ( 1 << temp ) ){
			//clear
			EXTI->PR1 |= ( 1 << temp );
		}
	}
	else{
		if(EXTI->PR2 & ( 1 << temp ) ){
			//clear
			EXTI->PR2 |= ( 1 << temp );
		}
	}

}
