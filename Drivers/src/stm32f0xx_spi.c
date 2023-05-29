/*
 * stm32f0xx_spi.c
 *
 *  Created on: Mar. 22, 2023
 *      Author: hanzahar
 */


#include "stm32f0xx_spi.h"



void SPI_PeriClockControl(SPI_Reg_t *SPIx, State sclkState)
{
	if (sclkState == ENABLE)
	{
		if (SPIx == SPI1)
			SPI1_CLK_EN();

		if (SPIx == SPI2)
			SPI2_CLK_EN();

	}
	else
	{
		if (SPIx == SPI1)
			SPI1_CLK_DISABLE();

		if (SPIx == SPI2)
			SPI2_CLK_DISABLE();
	}
}

void SPI_Init(SPI_Handle_t *SPIxHandle)
{
	//* Configure CR1 register
	uint8_t MSTR     = SPIxHandle->SPIConfig.DeviceMode;
	uint8_t BIDIMODE = SPIxHandle->SPIConfig.BusConfig;
	uint8_t BR 		 = SPIxHandle->SPIConfig.SclkSpeed;
	uint8_t DS		 = SPIxHandle->SPIConfig.DS;
	uint8_t CPHA  	 = SPIxHandle->SPIConfig.CPHA;
	uint8_t CPOL	 = SPIxHandle->SPIConfig.CPOL;



	// Device mode
	SPIxHandle->SPIx->CR1 |= (MSTR << 2);


	// Bus config
	if (BIDIMODE == SPI_BUS_CONFIG_FD)
		SPIxHandle->SPIx->CR1 &= ~(1 << 15); // 2-line unidirectional

	else if (BIDIMODE == SPI_BUS_CONFIG_HD)
		SPIxHandle->SPIx->CR1 |= (1 << 15);  // 1-line bidirectional

	else if (BIDIMODE == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// 1 line unidirectional (Rx only)
		SPIxHandle->SPIx->CR1 |= (1 << 15);
		SPIxHandle->SPIx->CR1 |= (1 << 10);
	}


	// Clock speed
	SPIxHandle->SPIx->CR1 |= (BR << 3);


	// Data frame format
	SPIxHandle->SPIx->CR2 |= (DS << 8);


	// CPOL
	SPIxHandle->SPIx->CR1 |= (CPOL << 1);

	// CPHA
	SPIxHandle->SPIx->CR1 |= (CPHA);


}

/**
 * @brief SPI enable/disable function
 * 
 * @param SPIx SPI register type pointer
 * @param enable ENABLE or DISABLE
 */
void SPI_Enable(SPI_Reg_t *SPIx, uint8_t enable)
{
    if (enable == ENABLE)
    {
        SPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        while (SPI_BUSY_FLAG(SPIx)); //* Wait for busy flag to be cleared

        SPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

/**
 * @brief 
 * 
 * @param SPIx 
 */
void SPI_DeInit(SPI_Reg_t *SPIx)
{

}


/**
 * @brief Returns SPix Status Register Flag status
 * 
 * @param SPIx SPIx register type ptr
 * @param flagName SPI related status flags definitions in stm32f0xx_spi.h
 * @return uint8_t 
 */
uint8_t SPI_GetFlagStatus(SPI_Reg_t *SPIx, uint32_t flagName)
{
    return (SPIx->SR & flagName);
}


/**
 * @brief Send data over SPIx
 * 
 * @param SPIx SPIx register type ptr
 * @param txBuffer Data array to send
 * @param dataLen Length of data buffer
 */
void SPI_sendData(SPI_Reg_t *SPIx, uint8_t *txBuffer, uint32_t dataLen)
{
    //* Check DFF
    bool eightBit;
    uint8_t DFF = (SPIx->CR2 >> 8) & 0x0F;
    
    if ( DFF & SPI_DFF_8BITS ) //< DFF is 8 bit
        eightBit = true;
    else if( DFF & SPI_DFF_16BITS)
        eightBit = false;


    //* Load all data from txBuffer into SPI DR
    while (dataLen > 0)
    {
        //* Check TX buffer
        // Wait until TXE == 1 (TX buffer is empty)
//        while(SPI_GetFlagStatus(SPIx, SPI_TXE_FLAG) == NOT_EMPTY);

        if (eightBit) // load 8-bit data
        {
            // Load data register with data
            SPIx->DR = *txBuffer;

            // decrement data length counter
            dataLen--;

            // increment pointer in data buffer
            txBuffer++;
        }
        else // Load 16-bit data
        {
            // Load data register with (16-bit) data
            SPIx->DR = *(uint16_t *)txBuffer;

            // Decrement data length counter (2 bytes loaded)
            dataLen -= 2;

            // Increment pointer (by 2 bytes) in data buffer
            (uint16_t *)txBuffer++;
        }
    }

}


/**
 * @brief Read data from SPIx
 * 
 * @param SPIx SPIx register type ptr
 * @param rxBuffer Data buffer array to store read data
 * @param dataLen Length of data buffer
 */
void SPI_readData(SPI_Reg_t *SPIx, uint8_t *rxBuffer, uint32_t dataLen)
{
    //! if data length is 0, leave
    if (!(dataLen > 0)) return;



    //* Check DFF
    bool eightBit;
    uint8_t dff = (SPIx->CR2 >> 8) & 0x0F;
    
    if ( dff & SPI_DFF_8BITS ) //< DFF is 8 bit
        eightBit = true;
    else if( dff & SPI_DFF_16BITS)
        eightBit = false;



    while (dataLen > 0)
    {
        //* Check RX buffer
        // Wait until RXNE == 1 (Rx buffer not empty)
        while(SPI_RXNE_FLAG(SPIx));

        if (eightBit) // 8-bit DFF
        {
            // Load byte into rxBuffer
            *rxBuffer = SPIx->DR;

            // Increment rxBuffer ptr
            rxBuffer++;

            // Decrement dataLen counter
            dataLen--;
        }
        else // 16-bit DFF
        {
            // Load 16-bit data into rxBuffer
            *(uint16_t *)rxBuffer = SPIx->DR;

            // Increment rxBuffer ptr (by 16-bits)
            (uint16_t *)rxBuffer++;

            // Decrement dataLen counter (by 16-bits)
            dataLen -= 2;
        }
    }
}


/**
 * @brief Send data 
 * 
 * @param pSPIHandle 
 * @param pTxBuffer 
 * @param Len 
 * @return uint8_t 
 */
void SPI_sendDataIT(SPI_Handle_t *SPIxHandle, uint8_t *txBuffer_, uint32_t txlen_)
{

    if (SPIxHandle->txState != SPI_BUSY_IN_TX)
    {
        SPIxHandle->txBuffer = txBuffer_;
        
        SPIxHandle->txLen = txlen_;

        SPIxHandle->txState = SPI_BUSY_IN_TX;

        //* Enable TXEIE control bit
        // generates interrupt when TXE set
        SPIxHandle->SPIx->CR2 |= (1 << TXEIE);
    }




}


/**
 * @brief 
 * 
 * @param SPIx 
 * @param rxBuffer 
 * @param dataLen 
 */
void SPI_readDataIT(SPI_Handle_t *SPIxHandle, uint8_t *rxBuffer_, uint32_t rxLen_)
{
    SPIxHandle->rxBuffer = rxBuffer_;

    SPIxHandle->rxLen = rxLen_;

    SPIxHandle->rxState = SPI_BUSY_IN_RX;

    //* Enable RXEIE control bit
    // generates interrupt when RXNE set
    SPIxHandle->SPIx->CR2 |= (1 << RXEIE);



}

/**
 * @brief 
 * 
 * @param IRQNumber 
 * @param EnorDi 
 */
void SPI_IRQInterruptConfig(uint8_t irqNo, State state)
{

}

/**
 * @brief 
 * 
 * @param IRQNumber 
 * @param IRQPriority 
 */
void SPI_IRQPriorityConfig(uint8_t irqNo, uint32_t irqPrio)
{

}


/**
 * @brief Determines what type of SPI interrupt triggered (RXEIE, TXEIE, ERRIE)
 * 
 * @param pHandle 
 */
void SPI_IRQHandler(SPI_Handle_t *SPIxHandle)
{
    SPI_Reg_t *SPIx_ = SPIxHandle->SPIx;


    //* TX interrupt
    if (SPI_TXE_FLAG(SPIx_) && SPI_TXE_FLAG(SPIx_))
    {
        SPI_sendData(SPIxHandle->SPIx, SPIxHandle->txBuffer, SPIxHandle->txLen);


        if (SPIxHandle->txLen <= 0)
        {
            if (SPIxHandle->SPIx->CR2 & (1 << TXEIE))
                SPIxHandle->SPIx->CR2 &= ~(1 << TXEIE); //* Clear TXEIE bit after txBuffer sent
            
            SPIxHandle->txState = SPI_READY;

            SPI_ApplicationEventCallback(SPIxHandle, SPI_EVENT_TX_CMPLT);
        }

    }         
    

    //* RX Interrupt
    else if (SPI_RXNE_FLAG(SPIx_) && SPI_RXNE_FLAG(SPIx_))   
    {
        SPI_readData(SPIxHandle->SPIx, SPIxHandle->rxBuffer, SPIxHandle->rxLen);

        if (SPIxHandle->rxLen <= 0)
        {
            if (SPIxHandle->SPIx->CR2 & (1 << RXEIE))
                SPIxHandle->SPIx->CR2 &= ~(1 << TXEIE); //* Clear TXEIE bit after txBuffer sent
            
            SPIxHandle->rxState = SPI_READY;

            SPI_ApplicationEventCallback(SPIxHandle, SPI_EVENT_RX_CMPLT);
        }
    }


    //! Error interrupt
    else if (SPI_ERR(SPIx_))      
    {
        if (SPIxHandle->txState != SPI_BUSY_IN_TX)
        {

        }

        SPI_ApplicationEventCallback(SPIxHandle, SPI_EVENT_OVR_ERR);
    }

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *SPIxHandle, uint8_t AppEv)
{

    switch (AppEv)
    {
        case SPI_EVENT_TX_CMPLT:
            break;
        
        case SPI_EVENT_RX_CMPLT:
            break;
        
        case SPI_EVENT_CRC_ERR:
            break;
        
        case SPI_EVENT_OVR_ERR:
            break;
    }

}