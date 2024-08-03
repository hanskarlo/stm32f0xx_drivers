/**
 * @file stm32f0xx_crc.h
 * 
 * @author www.github.com/hanskarlo
 * 
 * @brief CRC peripheral Hardware Abstraction Layer (HAL) library
 * header file for STM32F0xx devices.
 * 
 */



#ifndef INC_STM32F0XX_CRC_H_
#define INC_STM32F0XX_CRC_H_

#include "stm32f0xx.h"




/*
 * CRC_CR bit positions
*/
#define CRC_CR_RESET        0
#define CRC_CR_POLYSIZE     3
#define CRC_CR_REV_IN       5
#define CRC_CR_REV_OUT      7



/*
 * CRC reverse input
*/
typedef enum
{
    NOT_AFFECTED,
    BY_BYTE,
    BY_HALF_WORD,
    BY_WORD
}CRC_ReverseInput_t;

/*
 * CRC reverse input
*/
typedef enum
{
    NOT_REVERSED,
    REVERSED
}CRC_ReverseOutput_t;


/*
 * CRC bit length
*/
typedef enum
{
    CRC_32, 
    CRC_16, 
    CRC_8,  
    CRC_7,  
}CRC_BitLength_t;


const bool CRC_Init(CRC_BitLength_t CRCn, uint32_t poly, CRC_ReverseInput_t rev_in, CRC_ReverseOutput_t rev_out);


const uint32_t CRC_Calculate(uint8_t *data, uint32_t dataLen);







#endif