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
    if (I2Cx_Handle->I2C_Config.sclSpeed == I2C_SCL_SPEED_SM)

    // Program address


    // CCR calculation

}

void I2C_DeInit(I2C_Reg_t *I2Cx)
{

}



