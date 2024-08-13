#include "stm32f0xx_timer.h"
/**
 * @file stm32f0xx_timer.c
 * 
 * @author www.github.com/hanskarlo
 * 
 * @brief Timer peripheral Hardware Abstraction Layer (HAL) library
 * source file for STM32f0xx devices. Provides API for configuration of basic,
 * general purpose and advanced timer peripherals.
 * 
 */


static void TIM_PCLK_Ctrl(Timer_Reg_t *TIMx, State state)
{
    if (state == ENABLE)
    {
        if (TIMx == TIM6)
            TIM6_PCLK_ENABLE();
        if (TIMx == TIM7)
            TIM7_PCLK_DISABLE();
    }
    else if (state == DISABLE)
    {
        if (TIMx == TIM6)
            TIM6_PCLK_DISABLE();
        if (TIMx == TIM7)
            TIM7_PCLK_DISABLE();
    }
}

const bool TIM_INNIT(TIMx_Handle_t *TIMx_Handle)
{
    TIM_PCLK_Ctrl(TIMx_Handle->TIMx, ENABLE);

    // TODO initialize timer config
}