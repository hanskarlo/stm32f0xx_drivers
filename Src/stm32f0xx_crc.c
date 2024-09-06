/**
 * @file stm32f0xx_crc.c
 * 
 * @author www.github.com/hanskarlo
 * 
 * @brief CRC peripheral Hardware Abstraction Layer (HAL) library
 * source file for STM32F0xx devices. Provides support for initializing the
 * CRC calculation unit, programming the generator polynomial*, 
 * and calculating CRC32, CRC16*, and CRC8* codes.
 * 
 * ! STM32F07x and STM32F09x devices are the only ones capable of programming the polynomial.
 */


#include "stm32f0xx_crc.h"


static CRC_BitLength_t _crc_bit_length;

/**
 * @brief Enables/Disables CRC peripheral clock
 * 
 * @param enable
 */
static void CRC_PCLK_Ctrl(bool enable)
{
    if (enable)
        CRC_CLK_EN();
    else
        CRC_CLK_DISABLE();
}


/**
 * @brief Configures CRC peripheral clock and control parameters.
 * 
 * @warning Only STM32F07x and STM32F09x devices can program the
 * polynomial. 
 * 
 * @param CRCn 
 * @param poly Generator polynomial
 * @param reverse_input Reverse input data
 * @param reverse_output Reverse output data
 */
const bool CRC_Init(CRC_BitLength_t CRCn, uint32_t poly, CRC_ReverseInput_t rev_in, CRC_ReverseOutput_t rev_out)
{
    // Enable peripheral clock
    CRC_PCLK_Ctrl(ENABLE);

    // Reset CRC unit
    CRC->CRC_CR |= (1 << CRC_CR_RESET);

    // Set CRC polynomial size
    CRC->CRC_CR |= (CRCn << CRC_CR_POLYSIZE);
    _crc_bit_length = CRCn;

    // Set reverse input data control
    CRC->CRC_CR |= (CRCn << CRC_CR_REV_IN);

    // Set reverse output data control
    CRC->CRC_CR |= (CRCn << CRC_CR_REV_OUT);

    // Set polynomial
    CRC->CRC_POL = poly;

    return true;
}


/**
 * @brief Calculate CRCn code.
 * 
 * @param data Data buffer
 * @param dataLen Data length in bytes
 * @return const uint32_t 
 */
const uint32_t CRC_Calculate(uint8_t *data, uint32_t dataLen)
{
    // Reset CRC unit
    CRC->CRC_CR |= (1 << CRC_CR_RESET);


    //* Input data into CRC_DR
    //* ^^^^^^^^^^^^^^^^^^^^^^
    // Optimized to input as much data as possible to minimize writes.
    uint8_t i = 0;
    while (dataLen > 0)
    {
        if (dataLen / 32)
        {
            // Input 32 bits (4 bytes) of data
            CRC->CRC_DR = *(uint32_t *) &data[i];

            // Decrement data length byte counter
            dataLen -= 4;

            // Increment data byte pointer
            i += 4;
        }
        else if (dataLen / 16)
        {
            // Input 16 bits (2 bytes) of data
            *(uint16_t *)CRC->CRC_DR = *(uint16_t *) &data[i];
            dataLen -= 2;
            i += 2;
        }
        else
        {
            // Input 8 bits (1 byte) of data
            *(uint8_t *) CRC->CRC_DR = data[i];
            dataLen -= 1;
            i += 1;
        }
    }
    
    // Return CRC calculation
    return CRC->CRC_DR;
}








