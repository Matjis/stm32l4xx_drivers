/*
 * stm32l476xx.h
 *
 *  Created on: Dec 20, 2020
 *      Author: Matiss
 */

#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_

#include <stdint.h>

#define __vo volatile


// Processor specific details

// ARM Cortex Mx Processor NVIC ISERx register Addresses

#define NVIC_ISER0					( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1					( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2					( (__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3					( (__vo uint32_t*) 0xE000E10C )
#define NVIC_ISER4					( (__vo uint32_t*) 0xE000E110 )
#define NVIC_ISER5					( (__vo uint32_t*) 0xE000E114 )
#define NVIC_ISER6					( (__vo uint32_t*) 0xE000E118 )
#define NVIC_ISER7					( (__vo uint32_t*) 0xE000E11C )

// ARM Cortex Mx Processor NVIC ICERx register Addresses

#define NVIC_ICER0					( (__vo uint32_t*) 0xE000E180 )
#define NVIC_ICER1					( (__vo uint32_t*) 0xE000E184 )
#define NVIC_ICER2					( (__vo uint32_t*) 0xE000E188 )
#define NVIC_ICER3					( (__vo uint32_t*) 0xE000E18C )
#define NVIC_ICER4					( (__vo uint32_t*) 0xE000E190 )
#define NVIC_ICER5					( (__vo uint32_t*) 0xE000E194 )
#define NVIC_ICER6					( (__vo uint32_t*) 0xE000E198 )
#define NVIC_ICER7					( (__vo uint32_t*) 0xE000E19C )

// ARM Cortex Mx Processor Priority register address calculation

#define NVIC_PR_BASE_ADDR			( (__vo uint32_t*) 0xE000E400 )

// ARM Cortex Mx Processor number of priority bits implemented in Priority register

#define NO_PR_BITS_IMPLEMENTED		4


// Base addresses of Flash and SRAM memories. Can be checked in memory mapping section of reference manual

#define FLASH_BASEADDR 				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				0x10000000U
#define	SYS MEM						0x1FFF0000U
#define SRAM 						SRAM1-BASEADDR


//AHBx and PBx Bus peripheral base addresses

#define	PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define	APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x48000000U


// Base addresses of peripherals which are on APB1 bus

#define I2C1_BASEADDR				(APB1PERIPH_BASE + 0x5400) //APB1 base address + offset
#define I2C2_BASEADDR				(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR				(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR				(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASE + 0x4c00)
#define UART5_BASEADDR				(APB1PERIPH_BASE + 0x5000)
#define LPUART1_BASEADDR			(APB1PERIPH_BASE + 0x8000)
#define I2C4_BASEADDR				(APB1PERIPH_BASE + 0x8400)


// Base addresses of peripherals which are on APB2 bus

#define SYSCFG_BASEADDR				(APB2PERIPH_BASE + 0x0000) //APB2 base address + offset
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x0400)
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x3800)
#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000)


// Base addresses of peripherals which are on AHB1 bus

#define RCC_BASEADDR				(AHB1PERIPH_BASE + 0x1000)


// Base addresses of peripherals which are on AHB2 bus

#define GPIOA_BASEADDR				(AHB2PERIPH_BASE + 0x0000) //AHB2 base address + offset
#define GPIOB_BASEADDR				(AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR				(AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR				(AHB2PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR				(AHB2PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR				(AHB2PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR				(AHB2PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR				(AHB2PERIPH_BASE + 0x1C00)


// Peripheral register definition structure for GPIO

typedef struct{

	__vo uint32_t MODER; 											//Address offset 0x0000
	__vo uint32_t OTYPER;											//Address offset 0x0400
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;
	__vo uint32_t BRR;
	__vo uint32_t ASCR;

}GPIO_RegDef_t;


// Peripheral register definition structure for SPI

typedef struct{

	__vo uint32_t CR1; 											//Address offset 0x0000
	__vo uint32_t CR2;											//Address offset 0x0400
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;

}SPI_RegDef_t;


// Peripheral register definition structure for Reset and clock control (RCC)

typedef struct{

	__vo uint32_t CR;			/*				Address offset: 0x00 */
	__vo uint32_t ICSCR;		/*				Address offset: 0x04 */
	__vo uint32_t CFGR;			/*				Address offset: 0x08 */
	__vo uint32_t PLLCFGR;		/*				Address offset: 0x0C */
	__vo uint32_t PLLSAI1CFGR;	/*				Address offset: 0x10 */
	__vo uint32_t PLLSAI2CFGR;	/*				Address offset: 0x14 */
	__vo uint32_t CIER;			/*				Address offset: 0x18 */
	__vo uint32_t CIFR;			/*				Address offset: 0x1C */
	__vo uint32_t CICR;			/*				Address offset: 0x20 */
	uint32_t	  RESERVED0;		/*				Address offset: 0x24 */
	__vo uint32_t AHB1RSTR;		/*				Address offset: 0x28 */
	__vo uint32_t AHB2RSTR;		/*				Address offset: 0x2C */
	__vo uint32_t AHB3RSTR;		/*				Address offset: 0x30 */
	uint32_t	  RESERVED1;		/*				Address offset: 0x34 */
	__vo uint32_t APB1RSTR1;	/*				Address offset: 0x38 */
	__vo uint32_t APB1RSTR2;	/*				Address offset: 0x3C */
	__vo uint32_t APB2RSTR;		/*				Address offset: 0x40 */
	uint32_t	  RESERVED2;		/*				Address offset: 0x44 */
	__vo uint32_t AHB1ENR;		/*				Address offset: 0x48 */
	__vo uint32_t AHB2ENR;		/*				Address offset: 0x4C */
	__vo uint32_t AHB3ENR;		/*				Address offset: 0x50 */
	uint32_t	  RESERVED3;		/*				Address offset: 0x54 */
	__vo uint32_t APB1ENR1;		/*				Address offset: 0x58 */
	__vo uint32_t APB1ENR2;		/*				Address offset: 0x5C */
	__vo uint32_t APB2ENR;		/*				Address offset: 0x60 */
	uint32_t	  RESERVED4;		/*				Address offset: 0x64 */
	__vo uint32_t AHB1SMENR;	/*				Address offset: 0x68 */
	__vo uint32_t AHB2SMENR;	/*				Address offset: 0x6C */
	__vo uint32_t AHB3SMENR;	/*				Address offset: 0x70 */
	uint32_t	  RESERVED5;		/*				Address offset: 0x74 */
	__vo uint32_t APB1SMENR1;	/*				Address offset: 0x78 */
	__vo uint32_t APB1SMENR2;	/*				Address offset: 0x7C */
	__vo uint32_t APB2SMENR;	/*				Address offset: 0x80 */
	uint32_t	  RESERVED6;		/*				Address offset: 0x84 */
	__vo uint32_t CCIPR;		/*				Address offset: 0x88 */
	uint32_t	  RESERVED7;		/*				Address offset: 0x8C */
	__vo uint32_t BDCR;			/*				Address offset: 0x90 */
	__vo uint32_t CSR;			/*				Address offset: 0x94 */
	__vo uint32_t CRRCR;		/*				Address offset: 0x98 */
	__vo uint32_t CCIPR2;		/*				Address offset: 0x9C */

}RCC_RegDef_t;


// Peripheral register definition structure for System configuration controller (SYSCFG)

typedef struct{

	__vo uint32_t MEMRMP;		/*				Address offset: 0x00 */
	__vo uint32_t CFGR1;		/*				Address offset: 0x04 */
	__vo uint32_t EXTICR[4];	/*				Address offset: 0x08 - 0x14 */
	__vo uint32_t SCSR;			/*				Address offset: 0x18 */
	__vo uint32_t CFGR2;		/*				Address offset: 0x1C */
	__vo uint32_t SWPR;			/*				Address offset: 0x20 */
	__vo uint32_t SKR;			/*				Address offset: 0x24 */
	uint8_t	  EMPTY;			/*				Address offset: 0x28 - this was added so that next reg would move to 0x29 as per reference manual */
	__vo uint32_t SWPR2;		/*				Address offset: 0x29 */

}SYSCFG_RegDef_t;


// Peripheral register definition structure for external interrupt (EXTI)

typedef struct{

	__vo uint32_t IMR1;			/*				Address offset: 0x00 */
	__vo uint32_t EMR1;			/*				Address offset: 0x04 */
	__vo uint32_t RTSR1;		/*				Address offset: 0x08 */
	__vo uint32_t FTSR1;		/*				Address offset: 0x0C */
	__vo uint32_t SWIER1;		/*				Address offset: 0x10 */
	__vo uint32_t PR1;			/*				Address offset: 0x14 */
	__vo uint32_t IMR2;			/*				Address offset: 0x20 */
	__vo uint32_t EMR2;			/*				Address offset: 0x24 */
	__vo uint32_t RTSR2;		/*				Address offset: 0x28 */
	__vo uint32_t FTSR2;		/*				Address offset: 0x2C */
	__vo uint32_t SWIER2;		/*				Address offset: 0x30 */
	__vo uint32_t PR2;			/*				Address offset: 0x34 */

}EXTI_RegDef_t;



// Peripheral definition ( Peripheral base addresses typecasted to xxx_RegDef_t)

#define GPIOA						( (GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB						( (GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC						( (GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD						( (GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE						( (GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF						( (GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG						( (GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH						( (GPIO_RegDef_t*) GPIOH_BASEADDR)

#define SPI1						( (SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2						( (SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3						( (SPI_RegDef_t*) SPI3_BASEADDR)

#define RCC							( (RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI						( (EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG						( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR)


// Clock enable macros for GPIOx peripherals

#define GPIOA_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 7 ) )


// Clock enable macros for I2Cx peripherals

#define I2C1_PCLK_EN()		( RCC->APB1ENR1 |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR1 |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR1 |= ( 1 << 23 ) )
#define I2C4_PCLK_EN()		( RCC->APB1ENR2 |= ( 1 << 1 ) )


// Clock enable macros for SPIx peripherals

#define SPI1_PCLK_EN()		( RCC->APB2ENR  |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR1 |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR1 |= ( 1 << 15 ) )


// Clock enable macros for USARTx peripherals

#define USART1_PCLK_EN()	( RCC->APB2ENR  |= ( 1 << 14 ) )
#define USART2_PCLK_EN()	( RCC->APB1ENR1 |= ( 1 << 17 ) )
#define USART3_PCLK_EN()	( RCC->APB1ENR1 |= ( 1 << 18 ) )
#define UART4_PCLK_EN()		( RCC->APB1ENR1 |= ( 1 << 19 ) )
#define UART5_PCLK_EN()		( RCC->APB1ENR1 |= ( 1 << 20 ) )
#define LPUART1_PCLK_EN()	( RCC->APB1ENR2 |= ( 1 << 0 ) )


// Clock enable macros for SYSCFGx peripherals

#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 0 ) )


// Clock disable macros for GPIOx peripherals

#define GPIOA_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 7 ) )


// Clock disable macros for I2Cx peripherals

#define I2C1_PCLK_DI()		( RCC->APB1ENR1 &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR1 &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR1 &= ~( 1 << 23 ) )
#define I2C4_PCLK_DI()		( RCC->APB1ENR2 &= ~( 1 << 1 ) )


// Clock disable macros for SPIx peripherals

#define SPI1_PCLK_DI()		( RCC->APB2ENR  &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR1 &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR1 &= ~( 1 << 15 ) )


// Clock disable macros for USARTx peripherals

#define USART1_PCLK_DI()	( RCC->APB2ENR  &= ~( 1 << 14 ) )
#define USART2_PCLK_DI()	( RCC->APB1ENR1 &= ~( 1 << 17 ) )
#define USART3_PCLK_DI()	( RCC->APB1ENR1 &= ~( 1 << 18 ) )
#define UART4_PCLK_DI()		( RCC->APB1ENR1 &= ~( 1 << 19 ) )
#define UART5_PCLK_DI()		( RCC->APB1ENR1 &= ~( 1 << 20 ) )
#define LPUART1_PCLK_DI()	( RCC->APB1ENR2 &= ~( 1 << 0 ) )


// Clock disable macros for SYSCFGx peripherals

#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 0 ) )


//Macro to reset GPIOx peripheral

#define GPIOA_REG_RESET()	do { (RCC->AHB2RSTR |= ( 1 << 0 ) );   ( RCC->AHB2RSTR &= ~( 1 << 0 ) ); } while(0)
#define GPIOB_REG_RESET()	do { (RCC->AHB2RSTR |= ( 1 << 1 ) );   ( RCC->AHB2RSTR &= ~( 1 << 1 ) ); } while(0)
#define GPIOC_REG_RESET()	do { (RCC->AHB2RSTR |= ( 1 << 2 ) );   ( RCC->AHB2RSTR &= ~( 1 << 2 ) ); } while(0)
#define GPIOD_REG_RESET()	do { (RCC->AHB2RSTR |= ( 1 << 3 ) );   ( RCC->AHB2RSTR &= ~( 1 << 3 ) ); } while(0)
#define GPIOE_REG_RESET()	do { (RCC->AHB2RSTR |= ( 1 << 4 ) );   ( RCC->AHB2RSTR &= ~( 1 << 4 ) ); } while(0)
#define GPIOF_REG_RESET()	do { (RCC->AHB2RSTR |= ( 1 << 5 ) );   ( RCC->AHB2RSTR &= ~( 1 << 5 ) ); } while(0)
#define GPIOG_REG_RESET()	do { (RCC->AHB2RSTR |= ( 1 << 6 ) );   ( RCC->AHB2RSTR &= ~( 1 << 6 ) ); } while(0)
#define GPIOH_REG_RESET()	do { (RCC->AHB2RSTR |= ( 1 << 7 ) );   ( RCC->AHB2RSTR &= ~( 1 << 7 ) ); } while(0)


//Macro to reset SPIx peripheral

#define SPI1_REG_RESET()	do { (RCC->APB2RSTR |= ( 1 << 12 ) );   ( RCC->APB2RSTR &= ~( 1 << 12 ) ); } while(0)
#define SPI2_REG_RESET()	do { (RCC->APB1RSTR1 |= ( 1 << 14 ) );   ( RCC->APB1RSTR1 &= ~( 1 << 14 ) ); } while(0)
#define SPI3_REG_RESET()	do { (RCC->APB1RSTR1 |= ( 1 << 15 ) );   ( RCC->APB1RSTR1 &= ~( 1 << 15 ) ); } while(0)


//Return port code for given GPIOx base address
  //C conditional operators
#define GPIO_BASEADDR_TO_CODE(x)	( (x == GPIOA)?0:\
									  (x == GPIOB)?1:\
									  (x == GPIOC)?2:\
									  (x == GPIOD)?3:\
									  (x == GPIOE)?4:\
									  (x == GPIOF)?5:\
								 	  (x == GPIOG)?6:\
									  (x == GPIOH)?7:0)


/*Interrupt request (IRQ) numbers for this MCU. Can be seen in vector table.
 * For now here are only EXTI
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

//Macros for all the possible priority levels

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI15		15


//Some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET


//Bit possition definiton for SPI_CR1

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_CRCL		11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15


//Bit possition definiton for SPI_CR2

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_NSSP		3
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_CR2_DS			8
#define SPI_CR2_FRXTH		12
#define SPI_CR2_LDMA_RX		13
#define SPI_CR2_LDMA_TX		14


//Bit possition definiton for SPI_SR

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
#define SPI_SR_FRLVL		9
#define SPI_SR_FTLVL		11



#include "stm32l476xx_gpio_driver.h"
#include "stm32l476xx_spi_driver.h"

#endif /* INC_STM32L476XX_H_ */
