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


const bool TIM_Init(TIMx_Handle_t *TIMx_Handle)
{
    TIM_PCLK_Ctrl(TIMx_Handle->TIMx, ENABLE);

    // Set clock division
    if (TIMx_Handle->ClockDiv < TIMER_CLKDIV_1 || TIMx_Handle->ClockDiv > TIMER_CLKDIV_4)
        return false;
    else
        TIMx_Handle->TIMx->CR1 |= (TIMx_Handle->ClockDiv << TIM_CR1_CKD);

    // Set prescaler
    TIMx_Handle->TIMx->PSC = TIMx_Handle->Prescaler;

    // Set period (auto reload value)
    TIMx_Handle->TIMx->ARR = TIMx_Handle->Period;

    // Set Auto Reload Preload 
    if (TIMx_Handle->AutoReloadUpdateMode == INSTANT)
        TIMx_Handle->TIMx->CR1 &= ~(1 << TIM_CR1_ARPE);
    else if (TIMx_Handle->AutoReloadUpdateMode == ON_UPDATE)
        TIMx_Handle->TIMx->CR1 |= (1 << TIM_CR1_ARPE);
    
    return true;
}


void TIM_Start(TIMx_Handle_t *TIMx_Handle)
{
    TIMx_Handle->TIMx->CR1 |= (1 << TIM_CR1_CEN);
}


void TIM_Stop(TIMx_Handle_t *TIMx_Handle)
{
    TIMx_Handle->TIMx->CR1 &= ~(1 << TIM_CR1_CEN);
}


const bool TIM_DeInit(TIMx_Handle_t *TIMx_Handle)
{
    TIM_Stop(&TIMx_Handle);

    TIM_PCLK_Ctrl(TIMx_Handle->TIMx, DISABLE);

    return true;
}


void TIM_SetPeriod(TIMx_Handle_t *TIMx_Handle, uint16_t period)
{
    TIMx_Handle->TIMx->ARR = period;
}


const bool TIM_IRQ_Config(TIMx_Handle_t *TIMx_Handle, State state, uint8_t priority)
{
    // Get IRQ position based on TIMx
    uint8_t irq_num = 0;
    if (TIMx_Handle->TIMx == TIM2)
        irq_num = TIM2_IRQ_POS;
    else if (TIMx_Handle->TIMx == TIM3)
        irq_num = TIM3_IRQ_POS;
    else if (TIMx_Handle->TIMx == TIM6)
        irq_num = TIM6_IRQ_POS;
    else if (TIMx_Handle->TIMx == TIM7)
        irq_num = TIM7_IRQ_POS;
    else if (TIMx_Handle->TIMx == TIM14)
        irq_num = TIM14_IRQ_POS;
    else if (TIMx_Handle->TIMx == TIM15)
        irq_num = TIM15_IRQ_POS;
    else if (TIMx_Handle->TIMx == TIM16)
        irq_num = TIM16_IRQ_POS;
    else if (TIMx_Handle->TIMx == TIM17)
        irq_num = TIM17_IRQ_POS;
    else return false;
    

    // Check if valid interrupt priority
    if (priority < IRQ_PRIO_0 || priority > IRQ_PRIO_192)
        return false;


    // Configure interrupt priority
	uint8_t IPR_No = irq_num / 4;
	uint8_t byte_offset = irq_num % 4;
	*((uint8_t *)NVIC_IPR0 + (IPR_No * 4)) |= ( priority << ((8 * byte_offset) + 6) );


    // Enable interrupt in NVIC
    if (state == ENABLE)
        *NVIC_ISER |= (1 << irq_num);
    else if (state == DISABLE)
        *NVIC_ICER |= (1 << irq_num);
    else
        return false;


    return true;
}