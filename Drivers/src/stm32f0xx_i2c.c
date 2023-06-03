/**
 * 
 * @file stm32f0xx_i2c.c
 * 
 * @author github.com/hanskarlo
 * 
 * @brief 
 * 
 * @date 2023-05-29
 */

#include "stm32f0xx_i2c.h"


//**
 * @brief 
 * 
 * @param I2Cx 
 * @param enable 
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
        else if (I2Cx == I2Cx)
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
 * @brief 
 * 
 * @param I2Cx_Handle 
 */
void I2C_Init(I2C_Handle_t *I2Cx_Handle)
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
    

    // Program address
    I2Cx_Handle->I2Cx->OAR1 |= (1 << 15); // Disable Own Address1
    I2Cx_Handle->I2Cx->OAR1 |= (1 << 10); // Enable 7-bit address
    I2Cx_Handle->I2Cx->OAR1 |= (1 << 15); // Enable Own Address 1

    I2Cx_Handle->I2Cx->OAR1 |= (I2Cx_Handle->I2C_Config.deviceAddress << 1); // Set address in OA1[7:1]
}


/**
 * @brief 
 * 
 * @param I2Cx 
 */
void I2C_DeInit(I2C_Reg_t *I2Cx)
{

}


/**
 * @brief 
 * 
 * @param I2Cx_Handle 
 * @param txBuffer 
 * @param len 
 * @param slaveAddr 
 * @param Sr 
 */
void I2C_MasterSendData(I2C_Handle_t *I2Cx_Handle,uint8_t *txBuffer, uint8_t dataLen, uint8_t slaveAddr, uint8_t Sr)
{
    // Set addressing mode
    I2Cx_Handle->I2Cx->CR2 &= ~(1 << I2C_CR2_ADD10); // 7-bit

    // Set slave address
    I2Cx_Handle->I2Cx->CR2 |= (slaveAddr << 1);

    // Transfer direction
    I2Cx_Handle->I2Cx->CR2 &= ~(1 << I2C_CR2_RD_WRN); // Write

    // Number of bytes in send
    I2Cx_Handle->I2Cx->CR2 |= ~(dataLen << I2C_CR2_NBYTES);

    // Set AUTOEND; generate STOP automatically after NBYTES sent
    I2Cx_Handle->I2Cx->CR2 |= (1 << I2C_CR2_AUTOEND);

    //* Above params must be set before start generated
    // Generate START
    I2Cx_Handle->I2Cx->CR2 |= (1 << I2C_CR2_START);


    while (dataLen > 0)
    {
        while( !I2Cx_Handle->I2Cx->ISR & (1 << I2C_ISR_TXE) ); // Block till TXDR empty

        I2Cx_Handle->I2Cx->TXDR = *txBuffer;
        txBuffer++;
        dataLen--;
    }


}


/**
 * @brief 
 * 
 * @param I2Cx_Handle 
 * @param rxBuffer 
 * @param len 
 * @param slaveAddr 
 * @param Sr 
 */
void I2C_MasterReceiveData(I2C_Handle_t *I2Cx_Handle,uint8_t *rxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr)
{

}


