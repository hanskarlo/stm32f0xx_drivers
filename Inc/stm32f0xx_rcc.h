/**
 * @file stm32f0xx_rcc.h
 * 
 * @author github.com/hanskarlo
 * 
 * @brief 
 */

#ifndef INC_STM32F0XX_RCC_H_
#define INC_STM32F0XX_RCC_H_

#include "stm32f0xx.h"


#define HSI_FREQ_HZ     8000000U
#define HSI48_FREQ_HZ   48000000U


//* Register bit positions
//  ^^^^^^^^^^^^^^^^^^^^^^

/*
 * Clock configuration register (RCC_CFGR)
*/
#define RCC_CFGR_SW         0
#define RCC_CFGR_SWS        2
#define RCC_CFGR_HPRE       4
#define RCC_CFGR_PPRE       8
#define RCC_CFGR_PRE        14
#define RCC_CFGR_PLL_SRC    15
#define RCC_CFGR_PLL_XTRE   17
#define RCC_CFGR_PLL_MUL    18
#define RCC_CFGR_MCO        24
#define RCC_CFGR_MCOPRE     28
#define RCC_CFGR_PLL_NODIV  31


/*
 * System clock switch status values
*/
#define RCC_HSI_USED    0
#define RCC_HSE_USED    1
#define RCC_PLL_USED    2
#define RCC_HSI48_USED  3


/*
 * PLL input clock source values
*/
#define PLL_SRC_HSI_DIV_2   0 // Only on F03x and F05x
#define PLL_SRC_HSI         1
#define PLL_SRC_HSE         2
#define PLL_SRC_HSI48       3


// Sets the HSE value
void RCC_Set_HSE(uint32_t HSE_freq);

//This returns the PCLK clock value
uint32_t RCC_Get_PCLK(void);

// Returns PLLCLK
uint32_t RCC_Get_PLLCLK(void);


#endif