/**
 * @file stm32f0xx_rcc.c
 * 
 * @author your name (you@domain.com)
 * 
 * @brief 
 * 
 * @date 2023-06-01
 */



#include "stm32f0xx_rcc.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_PreScaler[4] = {2, 4 , 8, 16};


/**
 * @brief 
 * 
 * @return uint32_t 
 */
uint32_t RCC_GetPCLKValue(void)
{
    uint32_t sysClk = 0;
    uint8_t SWS = (RCC->CFGR >> 2) & 0x03;

    // Clock source
    if (SWS == 0 || SWS == 1) //! HSI and HSE both 8MHz on STM32F072RB-Disco
        sysClk = 8000000;
    else if (SWS == 2)
        sysClk = RCC_GetPLLOutputClock();


    // AHB Prescaler
    uint8_t HPRE = (RCC->CFGR >> 4) & 0x0F;
    uint8_t ahbPrescaler;
    if (HPRE < 8)
        ahbPrescaler = 1;
    else
        ahbPrescaler = AHB_PreScaler[HPRE - 8];
    

    // APB Prescaler 
    uint8_t PPRE = (RCC->CFGR >> 8) & 0x07;
    uint8_t apbPrescaler;
    if (PPRE < 4)
        apbPrescaler = 1;
    else
        apbPrescaler = APB_PreScaler[PPRE - 8];

    // Calculate PCLK
    uint32_t PCLK = (sysClk / ahbPrescaler) / apbPrescaler;

    return PCLK;
}


/**
 * @brief 
 * 
 * @return uint32_t 
 */
uint32_t  RCC_GetPLLOutputClock(void)
{
    return 0;
}