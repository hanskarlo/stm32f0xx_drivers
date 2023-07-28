/**
 * @file stm32f0xx_uart.c
 * 
 * @author your name (you@domain.com)
 * 
 * @brief 
 * 
 * @date 2023-06-05
 */



#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"



/**
 * @brief 
 * 
 * @param USARTx 
 * @param enable 
 */
void USART_PeriClockControl(USART_Reg_t *USARTx, State enable)
{
    if (enable == ENABLE)
    {
        if (USARTx == USART1)
            USART1_CLK_EN();
        else if (USARTx == USART2)
            USART2_CLK_EN();
        else if (USARTx == USART3)
            USART3_CLK_EN();
        else if (USARTx == USART4)
            USART4_CLK_EN();
        else if (USARTx == USART5)
            USART5_CLK_EN();
        else if (USARTx == USART6)
            USART6_CLK_EN();
        else if (USARTx == USART7)
            USART7_CLK_EN();
        else if (USARTx == USART8)
            USART8_CLK_EN();
    }
    else if (enable == DISABLE)
    {
        if (USARTx == USART1)
            USART1_CLK_DISABLE();
        else if (USARTx == USART2)
            USART2_CLK_DISABLE();
        else if (USARTx == USART3)
            USART3_CLK_DISABLE();
        else if (USARTx == USART4)
            USART4_CLK_DISABLE();
        else if (USARTx == USART5)
            USART5_CLK_DISABLE();
        else if (USARTx == USART6)
            USART6_CLK_DISABLE();
        else if (USARTx == USART7)
            USART7_CLK_DISABLE();
        else if (USARTx == USART8)
            USART8_CLK_DISABLE();
    }
}


/**
 * @brief Initializes UART peripheral using USART_Reg_t (stm32f0xx_usart.h)
 * 
 * @note Tx/Rx enabled by default.
 * 
 * @param USARTxHandle 
 */
void USART_Init(USART_Handle_t *USARTxHandle)
{
    // Enable peripheral clock
    USART_PeriClockControl(USARTxHandle->USARTx, ENABLE);

    // Enable Tx & Rx
    USARTxHandle->USARTx->CR1 |= (1 << USART_CR1_TE) | (1 << USART_CR1_RE);
    
    // Set word length
    if (USARTxHandle->USART_Config.wordLen == USART_WORDLEN_8BITS)
    {
        USARTxHandle->USARTx->CR1 &= ~(1 << USART_CR1_M0) & ~(1 << USART_CR1_M1);
    }
    else if (USARTxHandle->USART_Config.wordLen == USART_WORDLEN_9BITS)
    {
        USARTxHandle->USARTx->CR1 &= ~(1 << USART_CR1_M1);
        USARTxHandle->USARTx->CR1 |= (1 << USART_CR1_M0);
    }

    // Set stop bits
    USARTxHandle->USARTx->CR2 |= USARTxHandle->USART_Config.numStopBits;

    // Set oversampling mode for baud rate generation
    USARTxHandle->USARTx->CR1 &= ~(1 << USART_CR1_OVER8);

    // Set baud rate
    uint32_t f_Clk = RCC_GetPCLKValue();
    uint32_t brrValue = ( f_Clk / USARTxHandle->USART_Config.baudRate);

    USARTxHandle->USARTx->BRR = brrValue;

}


/**
 * @brief Deinitializes USARTx
 * 
 * @warning Resets USARTx configurations
 * 
 * @param USARTxHandle 
 */
void USART_DeInit(USART_Handle_t *USARTxHandle)
{
    // Reset USART Configurations (CR1/2)
    // Reset CR2
    USARTxHandle->USARTx->CR2 &= ~(USART_CR1_RESET_VALUE);

    // Reset CR1 (preserving UE bit)
    USARTxHandle->USARTx->CR1 &= ~(USART_CR1_RESET_VALUE + 1);

    // Reset baud rate setting
    USARTxHandle->USARTx->BRR &= ~(USART_BRR_RESET_VALUE);

    // Clear UE
    USARTxHandle->USARTx->CR1 &= ~(1 << USART_CR1_UE);
}


/**
 * @brief Enables/Disables USARTx.
 * 
 * @note Does not clear USARTx settings (See USART_DeInit()).
 * 
 * @param USARTx 
 * @param enable 
 */
void USART_PeripheralControl(USART_Reg_t *USARTx, State enable)
{
	if(enable == ENABLE)
	{
		USARTx->CR1 |= (1 << USART_CR1_UE);
	}else
	{
		USARTx->CR1 &= ~(1 << USART_CR1_UE);
	}

}