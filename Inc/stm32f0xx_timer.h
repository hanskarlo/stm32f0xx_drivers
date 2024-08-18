/**
 * @file stm32f0xx_timer.h
 * 
 * @author www.github.com/hanskarlo
 * 
 * @brief Timer peripheral Hardware Abstraction Layer (HAL) library
 * header file for STM32F0xx devices.
 * 
 */



#ifndef INC_STM32F0XX_TIMER_H_
#define INC_STM32F0XX_TIMER_H_

#include "stm32f0xx.h"



/*
 *  TIM Control Register 1 (CR1) bit positions
*/
#define TIM_CR1_CEN     0
#define TIM_CR1_UDIS    1
#define TIM_CR1_URS     2
#define TIM_CR1_OPM     3
#define TIM_CR1_DIR     4
#define TIM_CR1_CMS     5
#define TIM_CR1_ARPE    7 
#define TIM_CR1_CKD     8



#define TIM1_BRK_UP_TRG_COM_IRQ_POS 13
#define TIM1_CC_IRQ_POS             14
#define TIM2_IRQ_POS                15
#define TIM3_IRQ_POS                16
#define TIM6_IRQ_POS                17
#define TIM7_IRQ_POS                18
#define TIM14_IRQ_POS               19
#define TIM15_IRQ_POS               20
#define TIM16_IRQ_POS               21
#define TIM17_IRQ_POS               22


typedef enum
{
    TIMER_CLKDIV_1,
    TIMER_CLKDIV_2,
    TIMER_CLKDIV_4
}TimerClkDiv_t;

typedef enum
{
    UP,
    DOWN,
}TimerCounterMode_t;

typedef enum
{
    INSTANT,
    ON_UPDATE
}TimerAutoReloadMode_t;


typedef struct TIMx_Handle_t
{
    TimerClkDiv_t ClockDiv;
    TimerCounterMode_t CounterMode;
    TimerAutoReloadMode_t AutoReloadUpdateMode;
    uint16_t Prescaler;
    uint16_t Period;
    Timer_Reg_t *TIMx;
}TIMx_Handle_t;




const bool TIM_Init(TIMx_Handle_t *TIMx_Handle);
void TIM_DeInit(TIMx_Handle_t *TIMx_Handle);

void TIM_Start(TIMx_Handle_t *TIMx_Handle);
void TIM_Stop(TIMx_Handle_t *TIMx_Handle);

void TIM_SetPeriod(TIMx_Handle_t *TIMx_Handle, uint16_t period);

const bool TIM_IRQ_Config(TIMx_Handle_t *TIMx_Handle, State state, uint8_t priority);

#endif