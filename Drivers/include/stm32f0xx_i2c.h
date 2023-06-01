/**
 * 
 * @file stm32f0xx_i2c.h
 * 
 * @author github.com/hanskarlo
 * 
 * @brief 
 * 
 * @date 2023-05-29
 */


#ifndef INC_STM32F0XX_I2C_H_
#define INC_STM32F0XX_I2C_H_


#include "stm32f0xx.h"


typedef struct 
{
    uint32_t sclSpeed;
    uint8_t deviceAddress;
    uint8_t ackControl;
    uint8_t fmDutyCycle;
}I2C_Config_t;

typedef struct 
{
    I2C_RegDef_t    *I2Cx;
    I2C_Config_t    I2C_Config;
    uint8_t         *txBuffer;
    uint8_t         *rxBuffer;
    uint32_t        txLen;
    uint32_t        rxLen;
}I2C_Handle_t;


#define I2C_READY       0
#define I2C_BUSY_IN_RX  1
#define I2C_BUSY_IN_TX  2

#define I2C_SCL_SPEED_SM    100000
#define I2C_SCL_SPEED_FM4K  400000
#define I2C_SCL_SPEED_FM2K  200000

#define I2C_ACK_ENABLE      1
#define I2C_ACK_DISABLE     0

#define I2C_FM_DUTY_16_2    1
#define I2C_FM_DUTY_2       0

#define I2C_FLAG_TXE
#define I2C_FLAG_RXNE
#define I2C_FLAG_SB
#define I2C_FLAG_OVR
#define I2C_FLAG_AF
#define I2C_FLAG_ARLO
#define I2C_FLAG_BERR
#define I2C_FLAG_STOPF
#define I2C_FLAG_ADD10
#define I2C_FLAG_BTF
#define I2C_FLAG_ADDR
#define I2C_FLAG_TIMEOUT

#define I2C_ADDR


#endif