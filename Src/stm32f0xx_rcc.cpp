/**
 * @file stm32f0xx_rcc.c
 * 
 * @author github.com/hanskarlo
 * 
 * @brief 
 * 
 */



#include "stm32f0xx_rcc.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_PreScaler[4] = {2, 4 , 8, 16};

static uint32_t _HSE_FREQ_HZ = HSI_FREQ_HZ;



void RCC_Set_HSE(uint32_t HSE_FREQ_HZ)
{
    _HSE_FREQ_HZ = HSE_FREQ_HZ;
}


/**
 * @brief Returns the PCLK
 * 
 * @warning Written for STM32F072B-Disco
 * 
 * @return uint32_t - PCLK
 */
uint32_t RCC_Get_PCLK(void)
{
    uint32_t SYSCLK = 0;
    uint8_t SWS = (RCC->CFGR >> RCC_CFGR_SWS) & 0x03U;

    // Clock source
    if (SWS == RCC_HSI_USED || SWS == RCC_HSE_USED) //! HSI and HSE both 8MHz on STM32F072RB-Disco
        SYSCLK = HSI_FREQ_HZ;
    else if (SWS == RCC_HSE_USED)
        SYSCLK = RCC_Get_PLLCLK();
    else if (SWS == RCC_HSI48_USED)
        SYSCLK = 48000000;


    // AHB Prescaler
    uint8_t HPRE = (RCC->CFGR >> RCC_CFGR_HPRE) & 0x0FU;
    uint8_t ahbPrescaler;
    if (HPRE < 8)
        ahbPrescaler = 1;
    else
        ahbPrescaler = AHB_PreScaler[HPRE - 8];
    

    // APB Prescaler 
    uint8_t PPRE = (RCC->CFGR >> RCC_CFGR_PPRE) & 0x07U;
    uint8_t apbPrescaler;
    if (PPRE < 4)
        apbPrescaler = 1;
    else
        apbPrescaler = APB_PreScaler[PPRE - 8];


    // Calculate PCLK
    uint32_t PCLK = (SYSCLK / ahbPrescaler) / apbPrescaler;

    return PCLK;
}


/**
 * @brief Returns Phased-Lock Loop clock (PLLCLK) rate in Hz.
 * 
 * @return uint32_t 
 */
uint32_t  RCC_Get_PLLCLK(void)
{
    // Determine PLL input clock source
    uint8_t PLLSRC = (RCC->CFGR >> RCC_CFGR_PLL_SRC) & 0xFFU;
    uint32_t PLLCLK;    

    switch (PLLSRC)
    {
        case PLL_SRC_HSI_DIV_2:
        {
            PLLCLK = (HSI_FREQ_HZ / 2);
            break;
        }

        case PLL_SRC_HSI:
        {
            PLLCLK = HSI_FREQ_HZ;
            break;
        }

        case PLL_SRC_HSE:
        {
            PLLCLK = _HSE_FREQ_HZ;
            break;
        }

        case PLL_SRC_HSI48:
        {
            PLLCLK = HSI48_FREQ_HZ;
            break;
        }

        default:
            PLLCLK = 0;
    }

    return PLLCLK;
}