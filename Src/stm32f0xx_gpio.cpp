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
void GPIO_PeriphClkCtrl(GPIO_Reg_t* GPIOx, State clkState)
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
 * @brief Read digital pin
 *
 * @param GPIO_Reg_t -- GPIO port register map struct
 * @param uint8_t pinNo -- Pin number (0-15)
 */
uint8_t GPIO_ReadPin(GPIO_Reg_t *GPIOx, uint8_t pinNo)
{
	typedef uint8_t byte;

	byte val = ( (GPIOx->IDR) >> pinNo ) & 0x00000001;

	return val;
}




/**
 * @brief Write digital pin to state
 *
 * @param GPIO_Reg_t GPIOx register map struct (stm32f0xx_gpio.h)
 * @param pinNo Pin number (0-15)
 * @param value Pin binary logic level (0,1)
 */
void GPIO_WritePin(GPIO_Reg_t* GPIOx, uint8_t PinNo, GPIOPinState pinState)
{
	if (pinState == HIGH)
		GPIOx->ODR |= (1 << PinNo);
	else
		GPIOx->ODR |= ~(1 << PinNo);
}


/**
 * @brief Write entire Portx state (x = A to F).
 * 
 * @param GPIOx GPIOx register map struct (stm32f0xx_gpio.h)
 * @param regValue Port logic level (0x0000, 0xFFFF)
 */
void GPIO_WritePort(GPIO_Reg_t* GPIOx, uint16_t regValue)
{
	GPIOx->ODR = regValue;
}


/**
 * @brief Toggle pin
 * 
 * @param GPIOx GPIOx register map struct (stm32f0xx_gpio.h)
 * @param PinNo Pin number
 */
void GPIO_TogglePin(GPIO_Reg_t* GPIOx, uint8_t pinNo)
{
	GPIOx->ODR ^= (1 << pinNo);
}


/**
 * @brief Set IRQ Config
 *
 * @param irqNo -- IRQ number (1 - EXTI[0:1], 2 - EXTI[2:3], 3 - EXTI[4:15])
 * @param irq_priority -- IRQ priority (0-3, representing priority value 0-192 in steps of 64)
 * @param toggle -- ENABLE or DISABLE interrupt
 */
bool GPIO_IRQConfig(uint8_t irqNo, uint8_t irq_priority, State toggle)
{
    // Check IRQ number cooresponds to assigned vector
    if ((irqNo != IRQ_POS_EXTI0_1) || (irqNo != IRQ_POS_EXTI2_3) || (irqNo != IRQ_POS_EXTI2_3))
        return false;


	if (toggle == ENABLE)
		*NVIC_ISER |=  (1 << irqNo);
	else
		*NVIC_ICER &= ~(1 << irqNo);


	// Configure interrupt priority
	uint8_t IPR_No = irqNo / 4;
	uint8_t byteOffset = irqNo % 4;

	*((uint8_t *)NVIC_IPR0 + (IPR_No * 4)) |= ( irq_priority << ((8 * byteOffset) + 6) );

    return true;
}


/**
 * @brief 
 * 
 */
void GPIO_IRQHandler()
{
    // TODO: IRQ Handler
}



