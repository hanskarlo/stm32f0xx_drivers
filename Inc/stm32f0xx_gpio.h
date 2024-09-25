/**
 * @file stm32f0xx_gpio.h
 * 
 * @author www.github.com/hanskarlo
 * 
 * @brief GPIO periphal Hardware Abstaction Layer (HAL) library
 * header for STM32F0xx devices. Provides support for initialization,
 * I/O, and interrupt configuration.
 * 
 */

#ifndef INC_STM32F0XX_GPIO_H_
#define INC_STM32F0XX_GPIO_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx.h"





typedef enum
{
	GPIO_LOW,
	GPIO_HIGH
}GPIO_PinState_t;



/*
 * GPIOx Pin Config
 */
typedef struct
{
	uint8_t GPIO_PinNo;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPd;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunc;
}GPIO_PinConfig_t;




/*
 * Handle structure for a GPIOx pin
 */
typedef struct
{
	GPIO_Reg_t *GPIOx; /*!< Base address of GPIOx peripheral >*/
	GPIO_PinConfig_t GPIO_Config;
}GPIO_Handle_t;




/*
 * GPIO Pin No.
 */
#define GPIO_0	0
#define GPIO_1	1
#define GPIO_2	2
#define GPIO_3	3
#define GPIO_4	4
#define GPIO_5	5
#define GPIO_6	6
#define GPIO_7	7
#define GPIO_8	8
#define GPIO_9	9
#define GPIO_10	10
#define GPIO_11	11
#define GPIO_12	12
#define GPIO_13	13
#define GPIO_14	14
#define GPIO_15	15




/*
 * GPIO Port Configs
 */
#define INPUT			0
#define OUTPUT			1
#define ALT				2
#define ANALOG			3
#define INT_FALL		4
#define INT_RISE		5
#define INT_CHANGE   	6

/*
 * Output Type
 */
#define GPIO_PUSHPULL	0
#define GPIO_OPENDRAIN	1

/*
 * Output Speed
 */
#define LOW		0
#define MEDIUM	1
#define FAST    2
#define HIGH	3

/*
 * Pull Up/Down Config
 */
#define NONE 		0
#define PULL_UP		1
#define PULL_DOWN	2

/*
 * Alternate functions
 */
#define GPIO_AF0    0
#define GPIO_AF1	1
#define GPIO_AF2	2
#define GPIO_AF3	3
#define GPIO_AF4	4
#define GPIO_AF5	5
#define GPIO_AF6	6
#define GPIO_AF7	7




/***********************************
 * APIs
 **********************************/

/*
 * Initialize GPIOx
 */
void GPIO_Init(GPIO_Handle_t *GPIOHandle);

/*
 * Deinit GPIOx
 */
void GPIO_DeInit(GPIO_Reg_t *GPIOx);

/*
 * Set/Disable GPIOx clock
 */
void GPIO_PCLK_Ctrl(GPIO_Reg_t* GPIOx, State clkState);

/*
 * Read GPIOx pin
 */
uint8_t GPIO_ReadPin(GPIO_Handle_t *GPIOx);

/*
 * Read GPIOx port
 */
uint16_t GPIO_ReadPort(GPIO_Reg_t* GPIOx);

/*
 * Write to GPIOx pin
 */
void GPIO_WritePin(GPIO_Handle_t* GPIOx, GPIO_PinState_t pinState);

/*
 * Write to GPIOx port
 */
void GPIO_WritePort(GPIO_Reg_t* GPIOx, uint16_t regValue);

/*
 * Toggle GPIOx output pin
 */
void GPIO_TogglePin(GPIO_Handle_t* GPIOx);

/*
 * IRQ config and ISR handling
 */
const bool GPIO_IRQConfig(uint8_t IRQ_No, uint8_t IRQ_Prio, State toggle);








#endif /* INC_STM32F0XX_GPIO_H_ */
