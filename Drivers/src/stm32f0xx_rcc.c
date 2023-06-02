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


uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t sysClk = 0;
    uint8_t PPRE = (RCC->CFGR >> 8) & 0x07;

    if (PPRE = 0x04)
        sysClk = 

}