/**
 * @file stm32f0xx_gpio.cpp
 * 
 * @author www.github.com/hanskarlo
 * 
 * @brief GPIO periphal Hardware Abstaction Layer (HAL) library
 * source file for STM32F0xx devices. Provides support for initialization,
 * I/O, and interrupt configuration.
 * 
 */

#include "stm32f0xx_gpio.h"


/**
 * @brief Configures GPIO peripheral clock
 * 
 * @param GPIOx GPIOx register type (stm32f0xx_gpio.h)
 * @param clkState ENABLE or DISABLE
 */
void GPIO_PCLK_Ctrl(GPIO_Reg_t* GPIOx, State clkState)
{
	if (clkState == ENABLE)
	{

		if (GPIOx == GPIOA)
			GPIOA_CLK_EN();

		if (GPIOx == GPIOB)
			GPIOB_CLK_EN();

		if (GPIOx == GPIOC)
			GPIOC_CLK_EN();

		if (GPIOx == GPIOD)
			GPIOD_CLK_EN();

		if (GPIOx == GPIOE)
			GPIOE_CLK_EN();

		if (GPIOx == GPIOF)
			GPIOF_CLK_EN();

	}
	else if (clkState == DISABLE)
	{

		if (GPIOx == GPIOA)
			GPIOA_CLK_DISABLE();

		if (GPIOx == GPIOB)
			GPIOB_CLK_DISABLE();

		if (GPIOx == GPIOC)
			GPIOC_CLK_DISABLE();

		if (GPIOx == GPIOD)
			GPIOD_CLK_DISABLE();

		if (GPIOx == GPIOE)
			GPIOE_CLK_DISABLE();

		if (GPIOx == GPIOF)
			GPIOF_CLK_DISABLE();

	}
}


/**
 * @brief Initialize GPIOx given settings configured in GPIO handle.
 * 
 * @param GPIOHandle GPIOx handle type (stm32f0xx_gpio.h) 
 */
void GPIO_Init(GPIO_Handle_t *GPIOHandle)
{
	// Mode
	uint8_t pinMode = GPIOHandle->GPIO_Config.GPIO_PinMode;
	uint8_t pinNo 	= GPIOHandle->GPIO_Config.GPIO_PinNo;
	uint8_t pinSpd 	= GPIOHandle->GPIO_Config.GPIO_PinSpeed;
	uint8_t pinPUPD = GPIOHandle->GPIO_Config.GPIO_PinPuPd;
	uint8_t pinOut 	= GPIOHandle->GPIO_Config.GPIO_PinOPType;
	uint8_t pinAltFx 	= GPIOHandle->GPIO_Config.GPIO_PinAltFunc;


	if (pinMode <= ANALOG)
	{
		GPIOHandle->GPIOx->MODER &= ~(0x3 << (2 * pinNo));
		GPIOHandle->GPIOx->MODER |= pinMode << (2 * pinNo);
	}
	else
	{
		if (pinMode == INT_FALL)
		{
			// Set corresponding bit in FTSR
			EXTI->EXTI_FTSR |= (1 << pinNo);

			// Clear corresponding bit in RTSR
			EXTI->EXTI_RTSR &= ~(1 << pinNo);


		}
		else if (pinMode == INT_RISE)
		{
			// Set corresponding bit in RTSR
			EXTI->EXTI_RTSR |= (1 << pinNo);

			// Clear corresponding bit in RTSR
			EXTI->EXTI_FTSR &= ~(1 << pinNo);
		}
		else if (pinMode == INT_CHANGE)
		{
			// Set both corresponding bits in FTSR and RTSR
			EXTI->EXTI_FTSR |= (1 << pinNo);
			EXTI->EXTI_RTSR |= (1 << pinNo);
		}

		// Set source input for EXTIx external interrupt
		uint8_t regNum = (pinNo / 4);
		uint8_t portMaskPos = (4 * (pinNo % 4));
		uint8_t portCode = GPIO_PORT_CODE(GPIOHandle->GPIOx);
		SYSCFG->EXTICR[regNum] |= portCode << portMaskPos;
	}


	// Speed
	GPIOHandle->GPIOx->OSPEEDR &= ~(0x3 << (2 * pinNo));
	GPIOHandle->GPIOx->OSPEEDR |= pinSpd << (2 * pinNo);


	// Pullup/Pulldown (PUPD)
	GPIOHandle->GPIOx->PUPDR   &= ~(0x3 << (2 * pinNo));
	GPIOHandle->GPIOx->PUPDR   |= pinPUPD << (2 * pinNo);


	// Output type
	GPIOHandle->GPIOx->OTYPER  &= ~(0x1 << pinNo);
	GPIOHandle->GPIOx->OTYPER  |= pinOut << pinNo;


	// Alternate fxn
	if (pinMode == ALT)
	{
		uint8_t index = pinNo / 8;
		uint8_t bitPos = (pinNo % 8) * 4;

		GPIOHandle->GPIOx->AFR[index] &= ~(0xF << bitPos);

		GPIOHandle->GPIOx->AFR[index] |= pinAltFx << bitPos;
	}

}


/**
 * @brief Deinitializes GPIOx, resets I/O port in RCC_AHBRSTR.
 * 
 * @param GPIOx GPIOx register type (stm32f0xx_gpio.h)
 */
void GPIO_DeInit(GPIO_Reg_t *GPIOx)
{

	if (GPIOx == GPIOA)
		GPIOA_REG_RESET();

	if (GPIOx == GPIOB)
		GPIOB_REG_RESET();

	if (GPIOx == GPIOC)
		GPIOC_REG_RESET();

	if (GPIOx == GPIOD)
		GPIOD_REG_RESET();

	if (GPIOx == GPIOE)
		GPIOE_REG_RESET();

	if (GPIOx == GPIOF)
		GPIOF_REG_RESET();

}




/**
 * @brief 
 * 
 * @param GPIOx GPIOx handler struct (stm32f0xx_gpio.h)
 * @param pinNo uint8_t pin number
 * @return uint8_t 
 */
uint8_t GPIO_ReadPin(GPIO_Handle_t *GPIOx)
{
	uint8_t val = ( (GPIOx->GPIOx->IDR) >> GPIOx->GPIO_Config.GPIO_PinNo) & 0x01U;

	return val;
}




/**
 * @brief Write digital pin to state
 *
 * @param GPIOx GPIOx register map struct (stm32f0xx_gpio.h)
 * @param value Pin binary logic level (0,1)
 */
void GPIO_WritePin(GPIO_Handle_t* GPIOx, GPIO_PinState_t pinState)
{
	if (pinState == GPIO_HIGH)
		GPIOx->GPIOx->ODR |= (1 << GPIOx->GPIO_Config.GPIO_PinNo);
	else if (pinState == GPIO_LOW)
		GPIOx->GPIOx->ODR |= ~(1 << GPIOx->GPIO_Config.GPIO_PinNo);
}


/**
 * @brief Write entire Portx state (x = A to F).
 * 
 * @param GPIOx GPIOx handle struct (stm32f0xx_gpio.h)
 * @param regValue Port logic level (0x0000, 0xFFFF)
 */
void GPIO_WritePort(GPIO_Reg_t* GPIOx, uint16_t regValue)
{
	if (GPIOx == GPIOA)
		GPIOA->ODR |= regValue;

	else if (GPIOx == GPIOB)
		GPIOB->ODR |= regValue;

	else if (GPIOx == GPIOC)
		GPIOC->ODR |= regValue;

	else if (GPIOx == GPIOD)
		GPIOD->ODR |= regValue;

	else if (GPIOx == GPIOE)
		GPIOE->ODR |= regValue;

	else if (GPIOx == GPIOF)
		GPIOF->ODR |= regValue;

}


/**
 * @brief Toggle pin
 * 
 * @param GPIOx GPIOx handler struct (stm32f0xx_gpio.h)
 * @param PinNo Pin number
 */
void GPIO_TogglePin(GPIO_Handle_t* GPIOx)
{
	GPIOx->GPIOx->ODR ^= (1 << GPIOx->GPIO_Config.GPIO_PinNo);
}


/**
 * @brief Set IRQ Config
 *
 * @param irqNo -- IRQ number (1 - EXTI[0:1], 2 - EXTI[2:3], 3 - EXTI[4:15])
 * @param irq_priority -- IRQ priority (0-3, representing priority value 0-192 in steps of 64)
 * @param toggle -- ENABLE or DISABLE interrupt
 */
const bool GPIO_IRQConfig(uint8_t irqNo, uint8_t irq_priority, State toggle)
{
    // Check IRQ number corresponds to assigned vector
    if ((irqNo != EXTI0_1) || (irqNo != EXTI2_3))
        return false;
    

    // Check IRQ priority
    if (irq_priority < IRQ_PRIO_0 || irq_priority > IRQ_PRIO_192)
        return false;


    // Enable/Disable IRQ
	if (toggle == ENABLE)
    {
		*NVIC_ISER |=  (1 << irqNo);
    }
	else if (toggle == DISABLE)
    {
		*NVIC_ICER &= ~(1 << irqNo);
        return false;
    }
    else return false;


	// Configure interrupt priority
	uint8_t IPR_No = irqNo / 4;
	uint8_t byteOffset = irqNo % 4;

	*((uint8_t *)NVIC_IPR0 + (IPR_No * 4)) |= ( irq_priority << ((8 * byteOffset) + 6) );

    return true;
}
