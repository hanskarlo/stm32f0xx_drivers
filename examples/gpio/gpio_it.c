/**
 * @file gpio.c
 * 
 * @author www.github.com/hanskarlo
 * 
 * @brief Example for using the stm32f0xx GPIO HAL in an interrupt-based
 * configuration. Pin A0 is used as an interrupt which toggles the state of 
 * pin A1.
 * 
 */


#include "stm32f0xx_gpio.h"

GPIO_Handle_t intPin;
GPIO_Handle_t togglePin;

int main(void)
{
    // Configure pin being toggled
    togglePin.GPIOx = GPIOA;
    togglePin.GPIO_Config.GPIO_PinNo = GPIO_1;
    togglePin.GPIO_Config.GPIO_PinMode = OUTPUT;
    togglePin.GPIO_Config.GPIO_PinOPType = GPIO_PUSHPULL;
    togglePin.GPIO_Config.GPIO_PinPuPd = NONE;
    togglePin.GPIO_Config.GPIO_PinSpeed = FAST;

    // Configure interrupt pin A0 with 
    // internal pull up triggered on falling edge 
    intPin.GPIOx = GPIOA;
    intPin.GPIO_Config.GPIO_PinNo = GPIO_0;
    intPin.GPIO_Config.GPIO_PinSpeed = FAST;
    intPin.GPIO_Config.GPIO_PinOPType = GPIO_PUSHPULL;
    intPin.GPIO_Config.GPIO_PinPuPd = PULL_UP;
    intPin.GPIO_Config.GPIO_PinMode = INT_FALL;

    // Enable peripheral clock for GPIO port B
    GPIO_PCLK_Ctrl(GPIOA, ENABLE);

    // Initialize toggled pin, interrupt pin
    GPIO_Init(&togglePin);
    GPIO_Init(&intPin);

    // Configure interrupt EXTI for GPIO
    if (!GPIO_IRQConfig(&intPin, EXTI0_1, IRQ_PRIO_48, ENABLE)) return 0;

    // Initialize state of GPIO A1
    GPIO_WritePin(&togglePin.GPIOx, togglePin.GPIO_Config.GPIO_PinNo, LOW);
    
	while(true);
}



void EXTI0_1_IRQHandler(void)
{
    GPIO_TogglePin(&togglePin.GPIOx, togglePin.GPIO_Config.GPIO_PinNo);

    // In this example, clears pending bits on register EXTIPR for lines 0-1
    // In practice IRQ handler, should consider which EXTI line is being used, and what pin
    EXTI->PR &= ~(0xFFFFFFFC);
}