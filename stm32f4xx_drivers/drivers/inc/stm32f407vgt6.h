/*
 * stm32f406vgt6.h
 *
 *  Created on: Oct 12, 2023
 *      Author: OS
 */

#ifndef INC_STM32F407VGT6_H_
#define INC_STM32F407VGT6_H_

#include <stdint.h>

#define __vol volatile

/* Base address of Flash and SRAM memories*/

#define FLASH_BASEADDR   			0x08000000 //
#define SRAM1_BASEADDR   			0x20000000 // 112KB
#define SRAM2_BASEADDR				0x2001C000
#define ROM_BASEADDR				0x1FFF0000
#define SRAM 						SRAM1_BASEADDR




/*
 *  AHBx and APBx Bus Peripheral base address
 */
#define PERIPH_BASEADDR					0x4000 0000U // Start TIM2
#define APB1PERIPH_BASEADDR				PERIPH_BASE
#define APB2PERIPH_BASEADDR				0x40010000U // Start TIM1
#define AHB1PERIPH_BASEADDR				0x40020000U // Start GPIOA
#define AHB2PERIPH_BASEADDR				0x50000000U // USB OTG FS

/*
 * Base address of peripheral which are hanging on AHB1 bus
 * TODO: Complete for all other peripherals
 */

#define GPIOA_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0c00)
#define GPIOE_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1c00)
#define GPIOI_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2800)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO: Complete for all other peripherals
 */

#define I2C1_BASE       			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE      				(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE       			(APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASE					(AHB1PERIPH_BASE + 0x3800)
#define SPI3_BASE					(AHB1PERIPH_BASE + 0x3C00)

#define USART2_BASE					(AHB1PERIPH_BASE + 0x4400)
#define USART3_BASE					(AHB1PERIPH_BASE + 0x4800)
/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO: Complete for all other peripherals
 */

#define EXTI_BASE					(APB2PERIPH_BASE + 3C00)
#define SPI1_BASE					(APB2PERIPH_BASE + 3000)
#define SYSCFG_BASE					(APB2PERIPH_BASE + 3C00)
#define USART6_BASE					(APB2PERIPH_BASE + 1400)
#define USART1_BASE					(APB2PERIPH_BASE + 1000)



/****************************Peripheral Register Definition Structures***************************/
/*
 * Note:
 */

typedef struct {
	__vol uint32_t MODER;					/* GPIO port mode register,                  Address offset: 0x00*/
	__vol uint32_t OTYPER;					/* GPIO port output type register            Address offset: 0x04*/
	__vol uint32_t OSPEEDR;					/* GPIO port output speed register           Address offset: 0x08*/
	__vol uint32_t PUPDR;					/* GPIO port pull-up/pull-down register      Address offset: 0x0C*/
	__vol uint32_t IDR;						/* GPIO port input data register             Address offset: 0x10*/
	__vol uint32_t ODR;						/* GPIO port output data register            Address offset: 0x14*/
	__vol uint32_t BSRR;					/* GPIO port bit set/reset register          Address offset: 0x18*/
	__vol uint32_t LCKR;					/* GPIO port configuration lock register     Address offset: 0x1C*/
	__vol uint32_t AFR[2];					/* GPIO alternate function low register AFR[0] Address Offset: 0x20,
										       GPIO alternate function low register AFR[1] Address Offset: 0x24,*/

} GPIO_RegDef_t;

typedef struct {
	__vol uint32_t CR; 								/* Address offset: 0x00*/
	__vol uint32_t PLLCFGR; 						/* Address offset: 0x04*/
	__vol uint32_t CFGR; 							/* Address offset: 0x08*/
	__vol uint32_t CIR; 							/* Address offset: 0x0C*/
	__vol uint32_t AHB1RSTR; 						/* Address offset: 0x10*/
	__vol uint32_t AHB2RSTR; 						/* Address offset: 0x14*/
	__vol uint32_t AHB3RSTR; 						/* Address offset: 0x18*/
	uint32_t RESERVERD0;							/* Reserved, 0x1C*/

	__vol uint32_t APB1RSTR; 						/* Address offset: 0x20*/
	__vol uint32_t APB2RSTR; 						/* Address offset: 0x24*/
	uint32_t RESERVERD1[2];							/* Reserved, 0x28-0x2C*/

	__vol uint32_t AHB1ENR;							/* Address offset: 0x30*/
	__vol uint32_t AHB2ENR;							/* Address offset: 0x34*/
	__vol uint32_t AHB3ENR;							/* Address offset: 0x38*/

	uint32_t RESERVERD2;							/* Reserved, 0x3C*/
	__vol uint32_t APB1ENR;							/* Address offset: 0x40*/
	__vol uint32_t APB2ENR;							/* Address offset: 0x44*/
	uint32_t RESERVED3[2];							/* Reserved, 0x48 - 0x4C*/
	__vol uint32_t AHB1LPENR;						/* Address offset: 0x50*/
	__vol uint32_t AHB2LPENR;						/* Address offset: 0x54*/
	__vol uint32_t AHB3LPENR;						/* Address offset: 0x58*/
	uint32_t RESERVERD3;							/* Reserved, 0x5C*/
	__vol uint32_t APB1LPENR;						/* Address offset: 0x60*/
	__vol uint32_t APB2LPENR;						/* Address offset: 0x64*/
	uint32_t RESERVERD4;							/* Reserved, 0x68 - 0x6C*/
	__vol uint32_t BDCR;							/* Address offset: 0x70*/
	__vol uint32_t CSR;								/* Address offset: 0x74*/
	uint32_t RESERVERD5;							/* Reserved, 0x78-0x7C*/
	__vol uint32_t SSCGR;							/* Address offset: 0x80*/
	__vol uint32_t PLLI2SCFGR;						/* Address offset: 0x84*/
	__vol uint32_t PLLSAICFGR;						/* Address offset: 0x88*/
	__vol uint32_t DCKCFGR;							/* Address offset: 0x8C*/


} RCC_RegDef_t;


#define GPIOA 						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 						((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 						((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 						((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 						((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define RCC							((GPIO_RegDef_t*)RCC_BASEADDR)


/*
 * Để kích hoạt Clock thì chúng ta phải chạm đến các ngoại vi
 *
 * Clock enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 7 ))

/*
 * Clock enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 21 ))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 22 ))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 23 ))

/*
 * Clock enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 12 ))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 14 ))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 15 ))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 13 ))
/*
 * Clock enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN()		(RCC->APB2ENR |= ( 1 << 4 ))
#define USART2_PCCK_EN()		(RCC->APB1ENR |= ( 1 << 17 ))
#define USART3_PCCK_EN()		(RCC->APB1ENR |= ( 1 << 18 ))
#define USART4_PCCK_EN()		(RCC->APB1ENR |= ( 1 << 19 ))
#define USART5_PCCK_EN()		(RCC->APB1ENR |= ( 1 << 20 ))
#define USART6_PCCK_EN()		(RCC->APB2ENR |= ( 1 << 5 ))
/*
 * Clock enable Macros for SVSCFG peripherals
 */
#define SVSCFG_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 14 ))


/*
 * Clock disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 0 ))
/*
 * Clock disable Macros for I2Cx peripherals
 */


/*
 * Clock disable Macros for SPIx peripherals
 */





#endif /* INC_STM32F407VGT6_H_ */
