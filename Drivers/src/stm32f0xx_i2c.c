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

void I2C_DeInit(I2C_Reg_t *I2Cx)
{

}



