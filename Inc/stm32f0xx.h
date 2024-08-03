/*
 * stm32f0xx.h
 *
 *  Created on: Nov 20, 2022
 *      Author: hanzahar
 */

#ifndef INC_STM32F0XX_H_
#define INC_STM32F0XX_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define __weak __attribute__((weak))

/*
 * ARM Cortex M0 Processor NVIC registers
 */
#define NVIC_ISER		( (volatile uint32_t*)0xE000E100U )
#define NVIC_ICER		( (volatile uint32_t*)0xE000E180U )
#define NVIC_IPR0		( (volatile uint32_t*)0xE000E400U )
#define NVIC_IPR1		(NVIC_IPR0 + 0x4)
#define NVIC_IPR2		(NVIC_IPR1 + 0x4)
#define NVIC_IPR3		(NVIC_IPR2 + 0x4)
#define NVIC_IPR4		(NVIC_IPR3 + 0x4)
#define NVIC_IPR5		(NVIC_IPR4 + 0x4)
#define NVIC_IPR6		(NVIC_IPR5 + 0x4)
#define NVIC_IPR7		(NVIC_IPR6 + 0x4)





/**
 * Base addresses for main memories
 */
#define MAIN_FLASH_MEM_BASE_ADDR		0x08000000U
#define SRAM_BASE_ADDR					0x20000000U
#define ROM_ADDR						0x1FFFX800U


#define SRAM_SIZE			0x4000U




/**
 * Base addresses for APBx and AHBx
 */
#define APB_BASE_ADDR					0x40000000U
#define AHB1_BASE_ADDR					0x40020000U
#define AHB2_BASE_ADDR					0x48000000U




/**
 * Base addresses of peripherals on AHB1 bus
 */

#define DMA_PERIPH_ADDR						(AHB1_BASE_ADDR)
#define DMA2_PERIPH_ADDR					0x40020400U
#define RCC_PERIPH_ADDR						0x40021000U
#define FLASH_INTERFACE_PERIPH_ADDR			0x40022000U
#define CRC_PERIPH_ADDR						0x40023000U
#define TSC_PERIPH_ADDR						0x40024000U




/**
 * Base addresses of peripherals on AHB2 bus
 */

#define GPIOA_PERIPH_ADDR					(AHB2_BASE_ADDR)
#define GPIOB_PERIPH_ADDR					0x48000400U
#define GPIOC_PERIPH_ADDR					0x48000800U
#define GPIOD_PERIPH_ADDR					0x48000C00U
#define GPIOE_PERIPH_ADDR					0x48001000U
#define GPIOF_PERIPH_ADDR					0x48001400U
#define RCC_PERIPH_ADDR						0x40021000U




/**
 * Base addresses of peripherals on APB Bus
 */

#define SPI1_PERIPH_ADDR					0x40013000U
#define SPI2_PERIPH_ADDR					0x40003800U

#define I2C1_PERIPH_ADDR					0x40005400U
#define I2C2_PERIPH_ADDR					0x40005800U

#define USART1_PERIPH_ADDR					0x40013800U
#define USART2_PERIPH_ADDR					0x40004400U
#define USART3_PERIPH_ADDR					0x40004800U
#define USART4_PERIPH_ADDR					0x40004C00U
#define USART5_PERIPH_ADDR					0x40005000U
#define USART6_PERIPH_ADDR					0x40011400U
#define USART7_PERIPH_ADDR					0x40011800U
#define USART8_PERIPH_ADDR					0x40011C00U

#define EXTI_BASE_ADDR						0x40010400U
#define CRC_BASE_ADDR                       0x40023000U
#define SYSCFG_BASE_ADDR					0x40010000U

typedef enum
{
	ENABLE,
	DISABLE,
	SET = ENABLE,
	RESET = DISABLE
}State;

/**
 * Struct for GPIO register map
 */
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
	volatile uint32_t BRR;
}GPIO_Reg_t;





/**
 * Struct for RCC register map
 */
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBRSTR;
	volatile uint32_t CFGR2;
	volatile uint32_t CFGR3;
	volatile uint32_t CR2;
}RCC_Reg_t;



/*
 * Register struct for EXTI register
 */
typedef struct{
	volatile uint32_t EXTI_IMR;		// Interrupt mask register
	volatile uint32_t EXTI_EMR;		// Event mask register
	volatile uint32_t EXTI_RTSR;	// Rising trigger selection register
	volatile uint32_t EXTI_FTSR;	// Falling trigger selection register
	volatile uint32_t EXTI_SWIER;	// S/W INT event register
	volatile uint32_t PR;			// Pending register
}EXTI_Reg_t;



/*
 * CRC Register struct
*/
typedef struct{
    volatile uint32_t CRC_DR;
    volatile uint32_t CRC_IDR;
    volatile uint32_t CRC_CR;
    volatile uint32_t CRC_INIT;
    volatile uint32_t CRC_POL;
}CRC_Reg_t;



/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} SPI_Reg_t;

/*
 * I2C Register structure  
 */
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t TIMINGR;
    volatile uint32_t TIMEOUTR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t PECR;
    volatile uint32_t RXDR;
    volatile uint32_t TXDR;
}I2C_Reg_t;

/*
 * USART Register structure
 */
typedef struct 
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t BRR;
    volatile uint32_t GTPR;
    volatile uint32_t RTOR;
    volatile uint32_t RQR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t RDR;
    volatile uint32_t TDR;
}USART_Reg_t;


/**
 *
 */
typedef struct{
	volatile uint32_t CFGR1;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CFGR2;
	volatile uint32_t ITLINEx[31];
}SYSCFG_Reg_t;


/**
 * GPIO peripheral register typecasted to GPIO_Reg_t
 */
#define GPIOA 							((GPIO_Reg_t*)GPIOA_PERIPH_ADDR)
#define GPIOB 							((GPIO_Reg_t*)GPIOB_PERIPH_ADDR)
#define GPIOC 							((GPIO_Reg_t*)GPIOC_PERIPH_ADDR)
#define GPIOD 							((GPIO_Reg_t*)GPIOD_PERIPH_ADDR)
#define GPIOE 							((GPIO_Reg_t*)GPIOE_PERIPH_ADDR)
#define GPIOF 							((GPIO_Reg_t*)GPIOF_PERIPH_ADDR)


/*
 * SPI Peripheral register
 */
#define SPI1							((SPI_Reg_t*)SPI1_PERIPH_ADDR)
#define SPI2							((SPI_Reg_t*)SPI2_PERIPH_ADDR)


/*
 * I2C Peripheral register 
 */
#define I2C1            ((I2C_Reg_t *)I2C1_PERIPH_ADDR)
#define I2C2            ((I2C_Reg_t *)I2C2_PERIPH_ADDR)

/*
 * UART Peripheral register
*/
#define USART1           ((USART_Reg_t *) USART1_PERIPH_ADDR)
#define USART2           ((USART_Reg_t *) USART2_PERIPH_ADDR)
#define USART3           ((USART_Reg_t *) USART3_PERIPH_ADDR)
#define USART4           ((USART_Reg_t *) USART4_PERIPH_ADDR)
#define USART5           ((USART_Reg_t *) USART5_PERIPH_ADDR)
#define USART6           ((USART_Reg_t *) USART6_PERIPH_ADDR)
#define USART7           ((USART_Reg_t *) USART7_PERIPH_ADDR)
#define USART8           ((USART_Reg_t *) USART8_PERIPH_ADDR)

/*
 * RCC peripheral register typecasted to RCC_Reg_t
 */
#define RCC				((RCC_Reg_t *)RCC_PERIPH_ADDR)


/*
 * External interrupt
 */
#define EXTI			((EXTI_Reg_t *)EXTI_BASE_ADDR)

/*
 * CRC peripheral register typecasted to CRC_Reg_t struct
*/
#define CRC             ((CRC_Reg_t *)CRC_BASE_ADDR)


/*
 * System Configuration register
 */
#define SYSCFG			((SYSCFG_Reg_t *)SYSCFG_BASE_ADDR)


/*
 * Reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17)); } while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18)); } while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19)); } while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20)); } while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 21)); (RCC->AHBRSTR &= ~(1 << 21)); } while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); } while(0)



/**
 *  Macro for calculating port code from port x
 */
#define GPIO_PORT_CODE(x)		(x == GPIOA) ? 0x0 : \
								(x == GPIOB) ? 0x2 : \
								(x == GPIOC) ? 0x3 : \
								(x == GPIOD) ? 0x4 : \
								(x == GPIOE) ? 0x5 : \
								(x == GPIOF) ? 0x6 : 0

/**
 * EXTI Line IRQ Positions
 */
#define IRQ_POS_EXTI0_1			5
#define IRQ_POS_EXTI2_3			6
#define IRQ_POS_EXTI4_15		7



/**
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_CLK_EN()			RCC->AHBENR |= (1 << 17)
#define GPIOB_CLK_EN()			RCC->AHBENR |= (1 << 18)
#define GPIOC_CLK_EN()			RCC->AHBENR |= (1 << 19)
#define GPIOD_CLK_EN()			RCC->AHBENR |= (1 << 20)
#define GPIOE_CLK_EN()			RCC->AHBENR |= (1 << 21)
#define GPIOF_CLK_EN()			RCC->AHBENR |= (1 << 22)

#define GPIOA_CLK_DISABLE()		RCC->AHBENR &= ~(1 << 17)
#define GPIOB_CLK_DISABLE()		RCC->AHBENR &= ~(1 << 18)
#define GPIOC_CLK_DISABLE()		RCC->AHBENR &= ~(1 << 19)
#define GPIOD_CLK_DISABLE()		RCC->AHBENR &= ~(1 << 20)
#define GPIOE_CLK_DISABLE()		RCC->AHBENR &= ~(1 << 21)
#define GPIOF_CLK_DISABLE()		RCC->AHBENR &= ~(1 << 22)



/**
 * Clock enable macros for CRC peripheral
 */

#define CRC_CLK_EN()			(RCC->AHBENR |= (1 << 6))
#define CRC_CLK_DISABLE()	    (RCC->AHBENR &= ~(1 << 6))


/**
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_CLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()			(RCC->APB1ENR |= (1 << 22))

#define I2C1_CLK_DISABLE()	    (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DISABLE()	    (RCC->APB1ENR &= ~(1 << 22))




/**
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_CLK_EN()			RCC->APB2ENR |= (1 << 12)
#define SPI2_CLK_EN()			RCC->APB1ENR |= (1 << 14)

#define SPI1_CLK_DISABLE()	RCC->APB2ENR &= ~(1 << 12)
#define SPI2_CLK_DISABLE()	RCC->APB1ENR &= ~(1 << 14)




/**
 * Clock enable macros for USART peripherals
 */

#define USART1_CLK_EN()		(RCC->APB2ENR |= (1 << 14))
#define USART2_CLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define USART4_CLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define USART5_CLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_CLK_EN()		(RCC->APB2ENR |= (1 << 5))
#define USART7_CLK_EN()		(RCC->APB2ENR |= (1 << 6))
#define USART8_CLK_EN()		(RCC->APB2ENR |= (1 << 7))

#define USART1_CLK_DISABLE()	(RCC->APB2ENR |= (1 << 14))
#define USART2_CLK_DISABLE()	(RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_DISABLE()	(RCC->APB1ENR |= (1 << 18))
#define USART4_CLK_DISABLE()	(RCC->APB1ENR |= (1 << 19))
#define USART5_CLK_DISABLE()	(RCC->APB1ENR |= (1 << 20))
#define USART6_CLK_DISABLE()	(RCC->APB2ENR |= (1 << 5))
#define USART7_CLK_DISABLE()	(RCC->APB2ENR |= (1 << 6))
#define USART8_CLK_DISABLE()	(RCC->APB2ENR |= (1 << 7))








#endif /* INC_STM32F0XX_H_ */
