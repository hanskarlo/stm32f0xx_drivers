/**
 * 
 * @file stm32f0xx_i2c.c
 * 
 * @author github.com/hanskarlo
 * 
 * @brief I2C periphal Hardware Abstaction Layer (HAL) library
 * functions for STM32F0xx devices.
 * 
 */

#include "stm32f0xx_i2c.h"


/**
 * @brief Enable the I2Cx peripheral in the CR1 register.
 * 
 * @example I2C_PeripheralControl(I2Cx, ENABLE);
 * 
 * @param I2Cx I2Cx register type 
 * @param enable ENABLE or DISABLE
 */
void I2C_PeripheralControl(I2C_Reg_t *I2Cx, State enable)
{
    if (enable == ENABLE)
        I2Cx->CR1 |= (1 << I2C_CR1_PE);
    else if (enable == DISABLE)
        I2Cx->CR1 &= ~(1 << I2C_CR1_PE);
}

/**
 * @brief 
 * 
 * @param I2Cx 
 * @param enable 
 */
void I2C_PeriClockControl(I2C_Reg_t *I2Cx, State enable)
{
    if (enable == ENABLE)
    {
        if(I2Cx == I2C1)
            I2C1_CLK_EN();
        else if (I2Cx == I2C2)
            I2C2_CLK_EN();
    }
    else
    {
        if (I2Cx == I2C1)
            I2C1_CLK_DISABLE();
        else if (I2Cx == I2C2)
            I2C2_CLK_DISABLE();
    }
}


/**
 * @brief Initialize I2Cx peripheral.
 * 
 * @note Invoke after configuring corresponding GPIO.
 * 
 * @param I2Cx_Handle I2Cx handle type
 * @param master true if controlelr, false if agent
 */
void I2C_Init(I2C_Handle_t *I2Cx_Handle, bool controller)
{
    // Enable APB clock
    I2C_PeriClockControl(I2Cx_Handle->I2Cx, ENABLE);


    // Configure freq
    if (I2Cx_Handle->I2C_Config.sclSpeed == I2C_SCL_SPEED_SM) // Standard
        I2Cx_Handle->I2Cx->TIMINGR = I2C_TIMINGR_SM;
    else if (I2Cx_Handle->I2C_Config.sclSpeed == I2C_SCL_SPEED_FM) // Fast
        I2Cx_Handle->I2Cx->TIMINGR = I2C_TIMINGR_FM;
    else if (I2Cx_Handle->I2C_Config.sclSpeed == I2C_SCL_SPEED_FMP) // Fast mode+
        I2Cx_Handle->I2Cx->TIMINGR = I2C_TIMINGR_FMP;


    // Set AUTOEND; generate STOP automatically after NBYTES sent
    I2Cx_Handle->I2Cx->CR2 |= (1 << I2C_CR2_AUTOEND);


    // Setup as agent
    if (!controller)
    {
        // Program address
        I2Cx_Handle->I2Cx->OAR1 &= ~(1 << 15); // Disable Own Address1
        
        I2Cx_Handle->I2Cx->OAR1 |= (1 << 10); // Enable 7-bit address
        I2Cx_Handle->I2Cx->OAR1 |= (I2Cx_Handle->I2C_Config.deviceAddress << 1); // Set address in OA1[7:1]

        I2Cx_Handle->I2Cx->OAR1 |= (1 << 15); // Enable Own Address 1
    }
}


/**
 * @brief Deinitializes the I2Cx peripheral. disables peripheral clock,
 * then disables in CR1 register.
 * 
 * @todo Disable GPIO?
 * 
 * @param I2Cx I2Cx register type 
 */
void I2C_DeInit(I2C_Reg_t *I2Cx)
{
    I2C_PeriClockControl(I2Cx, DISABLE);
    I2C_PeripheralControl(I2Cx, DISABLE);
}


/**
 * @brief Transmit I2C data.
 * 
 * @warning Blocking.
 * 
 * @warning Can only send maximum of 255 bytes of data.
 * 
 * @todo Accomodate data sizes greater than 255 using RELOAD
 * 
 * @param I2Cx_Handle I2Cx handle type
 * @param txBuffer pointer to data buffer
 * @param dataLen data length in bytes
 * @param agentAddr slave address
 */
bool I2C_SendData(I2C_Handle_t *I2Cx_Handle, uint8_t *txBuffer, uint8_t dataLen, uint8_t agentAddr)
{
    // Set addressing mode  
    I2Cx_Handle->I2Cx->CR2 &= ~(1 << I2C_CR2_ADD10); // 7-bit

    // Set slave address
    I2Cx_Handle->I2Cx->CR2 |= (agentAddr << 1);

    // Transfer direction
    I2Cx_Handle->I2Cx->CR2 &= ~(1 << I2C_CR2_RD_WRN); // Write

    // Number of bytes in send
    I2Cx_Handle->I2Cx->CR2 |= ~(dataLen << I2C_CR2_NBYTES);

    //* Above params must be set before start generated
    // Generate START
    I2Cx_Handle->I2Cx->CR2 |= (1 << I2C_CR2_START);

    // Check if NACKF received after START
    if ((I2Cx_Handle->I2Cx->ISR) | (1 << I2C_ISR_NACKF))
        return false;

    // Write data into TXDR
    while (dataLen > 0)
    {
        while( !(I2Cx_Handle->I2Cx->ISR & (1 << I2C_ISR_TXIS)) ); // Block till TXDR empty

        I2Cx_Handle->I2Cx->TXDR = *txBuffer;
        txBuffer++;
        dataLen--;
    }

    // AUTOEND enabled: usually have to wait for TC to be set, then send STOP
    // if (!(I2Cx_Handle->I2Cx->ISR | (1 << I2C_ISR_TC)))
    //     return false;
    // else
    return true;
}


/**
 * @brief Receive data from agent.
 * 
 * @warning Blocking.
 * 
 * @param I2Cx_Handle I2Cx handle type (stm32f0xx_i2c.h)
 * @param rxBuffer Buffer to store rx bytes
 * @param dataLen Length of expected rx buffer
 * @param agentAddr Slave address 
 * @param Sr 
 */
void I2C_ReceiveData(I2C_Handle_t *I2Cx_Handle, uint8_t *rxBuffer, uint8_t dataLen, uint8_t agentAddr, uint8_t Sr)
{
    // Set addressing mode
    I2Cx_Handle->I2Cx->CR2 &= ~(1 << I2C_CR2_ADD10); // 7-bit

    // Set slave address
    I2Cx_Handle->I2Cx->CR2 |= (agentAddr << 1);

    // Transfer direction
    I2Cx_Handle->I2Cx->CR2 |= (1 << I2C_CR2_RD_WRN); // Read

    // Number of bytes in send
    I2Cx_Handle->I2Cx->CR2 |= ~(dataLen << I2C_CR2_NBYTES);

    //* Above params must be set before start generated
    // Generate START
    I2Cx_Handle->I2Cx->CR2 |= (1 << I2C_CR2_START);

    // Read data
    while (dataLen > 0)
    {
        while( !(I2Cx_Handle->I2Cx->ISR & (1 << I2C_ISR_RXNE)) ); // Block till RXDR not empty

        *rxBuffer = I2Cx_Handle->I2Cx->RXDR;
        rxBuffer++;
        dataLen--;
    }

    // AUTOEND enabled:
    // NACK automatically sent after last byte preceding STOP/RESTART condition
    // Usually have to follow those steps here
}


/**
 * @brief Enable/disable interrupt for I2Cx with settable priority.
 * 
 * @param I2Cx_Handle I2Cx handle type def
 * @param enable ENABLE or DISABLE
 * @param irq_priority Number from 0-3 representing priorities 0, 48, 96, and 192
 */
void I2C_IRQInterruptConfig(I2C_Handle_t I2Cx_Handle, State enable, uint8_t irq_priority)
{
    // Determine IRQ number based on I2Cx
    uint8_t irq_num;
    if (I2Cx_Handle.I2Cx == I2C1)
        irq_num = I2C1_IRQ_NUM;
    else if (I2Cx_Handle.I2Cx == I2C2)
        irq_num = I2C2_IRQ_NUM;

    // Enable/Disable corresponding interrupt in ISER/ICER
    if (enable == ENABLE)
        *NVIC_ISER |= (1 << irq_num);
    else if (enable == DISABLE)
        *NVIC_ICER |= (1 << irq_num);
    

    // Configure interrupt priority
	uint8_t ipr_num = irq_num / 4;
	uint8_t byteOffset = irq_num % 4;

    
	*((uint8_t *)NVIC_IPR0 + (ipr_num * 4)) |= ( irq_priority << ((8 * byteOffset) + 6) );
}