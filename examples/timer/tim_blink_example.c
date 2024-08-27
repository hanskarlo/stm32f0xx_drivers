/**
 * @file timer_example.c
 * 
 * @author www.github.com/hanskarlo
 * 
 * @brief Example for using the stm32f0xx I2C HAL in an interrupt-based
 * configuration.
 * 
 * This example assumes the following: 
 *  - internal 8MHz oscillator (HSI) as the clock source.
 *  - PCLK not prescaled by AHB, APB prescalers (i.e. PCLK is 8MHz)
 */


#include "stm32f0xx.h"

GPIO_Handle_t blinkPin;

int main(void)
{
    //* Initialize pin to toggle
    // Port A
    blinkPin.GPIOx = GPIOA;
    // Pin 1
    blinkPin.GPIO_Config.GPIO_PinNo = 1; 
    // Pin mode
    blinkPin.GPIO_Config.GPIO_PinMode = OUTPUT;
    // Output type : Open Drain
    blinkPin.GPIO_Config.GPIO_PinOPType = GPIO_OPENDRAIN;
    // Enable pull up resistors (internal)
    blinkPin.GPIO_Config.GPIO_PinPuPd = NONE;
    // Pin speed : Fast
    blinkPin.GPIO_Config.GPIO_PinSpeed = FAST;

    GPIO_PCLK_Ctrl(GPIOD, ENABLE);
    GPIO_Init(&blinkPin);
    GPIO_WritePin();



    //* Initialize timer to toggle pin
    TIMx_Handle_t blinkTimer;

    // Using general purpose timer (TIM2)
    blinkTimer.TIMx = TIM2;
    // Counter mode : UP
    blinkTimer.CounterMode = UP;
    // Clock division : /1
    blinkTimer.ClockDiv = TIMER_CLKDIV_1;
    // Auto reload : on update
    blinkTimer.AutoReloadUpdateMode = ON_UPDATE;

    //* Setup timer period to trigger counter overflow every 2s
    uint16_t timer_freq = 1000; // Desired timer frequency: 1000Hz, 1000 counts per second

    // Set timer frequency to 1KHz by prescaling 8MHz PCLK by 1000
    uint16_t prescaler = (RCC_Get_PCLK() / timer_freq) - 1;
    blinkTimer.Prescaler = prescaler;

    // Set counter period in auto reload register
    // 1KHz timer counts 1000 counts/second -- set ARR to (2000 - 1) to trigger
    // overflow every 2000 ticks
    uint16_t arr = (timer_freq * 2) - 1;
    blinkTimer.Period = arr;

    // Initialize and configure TIM2 using configuration above
    if (!TIM_Init(&blinkTimer)) return 0;

    // Configure TIM2 IRQ
    if (!TIM_IRQ_Config(&blinkTimer, ENABLE, IRQ_PRIO_0)) return 0;
    
    // Start timer
    TIM_Start(&blinkTimer);

    /* Loop forever */
	while(true);
}


/**
 * @brief Toggle pin on every TIM2 interrupt
 * 
 */
void TIM2_IRQHandler(void)
{
    GPIO_TogglePin(&blinkPin);
}