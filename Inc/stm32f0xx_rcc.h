/**
 * @file stm32f0xx_rcc.h
 * 
 * @author your name (you@domain.com)
 * 
 * @brief 
 * 
 * @date 2023-06-01
 */

#ifndef INC_STM32F0XX_RCC_H_
#define INC_STM32F0XX_RCC_H_

#include "stm32f0xx.h"


//This returns the APB1 clock value
uint32_t RCC_GetPCLKValue(void);

// Returns PLL clk
uint32_t  RCC_GetPLLOutputClock(void);

#endif