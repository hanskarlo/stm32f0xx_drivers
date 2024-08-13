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



typedef enum
{
    UP,
    DOWN,
}TimerCounterMode_t;



typedef struct 
{
    TimerCounterMode_t CounterMode;
    uint16_t Prescaler;
    uint8_t ClockDiv;
    uint16_t Period;
    State AutoReload;
    Timer_Reg_t *TIMx;
}TIMx_Handle_t;




const bool TIM_INIT(TIMx_Handle_t *TimerHandler);





#endif