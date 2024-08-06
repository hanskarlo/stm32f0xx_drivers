/**
 * @file stm32f0xx_spi.c
 * 
 * @author www.github.com/hanskarlo
 * 
 * @brief SPI peripheral Hardware Abstraction Layer (HAL) library
 * header file for STM32f0xx devices. 
 * 
 */

#include "stm32f0xx_spi.h"



/**
 * @brief Enable/Disable APB clock for SPIx.
 * 
 * @param SPIx SPIx register type (stm32f0xx_spi.h)
 * @param enable 
 */
void SPI_PCLK_Control(SPI_Reg_t *SPIx, State state)
{
	if (state == ENABLE)
	{
		if (SPIx == SPI1)
			SPI1_CLK_EN();

		if (SPIx == SPI2)
			SPI2_CLK_EN();

	}
	else if (state == DISABLE)
	{
		if (SPIx == SPI1)
			SPI1_CLK_DISABLE();

		if (SPIx == SPI2)
			SPI2_CLK_DISABLE();
	}
}


/**
 * @brief Initialize SPIx peripheral.
 * 
 * @warning Configure relevant GPIO for SPIx peripheral
 * beforing calling this.
 * 
 * @note Unidirectional configuration assumes RX only.
 * @note Configures DFF with MSB first.
 * 
 * @todo Add CRC support
 * @todo Add option for LSBFIRST config
 * @todo Add Slave configuration support
 * 
 * @param SPIx_Handle SPIx handle type (stm32f0xx_spi.h)
 * @return true successful initialization
 * @return false failed
 */
const bool SPI_Init(SPI_Handle_t *SPIx_Handle)
{
	//* Configure CR1 register
	uint8_t MSTR     = SPIx_Handle->SPIConfig.DeviceMode;
	uint8_t BIDIMODE = SPIx_Handle->SPIConfig.BusConfig;
	uint8_t BR 		 = SPIx_Handle->SPIConfig.SclkSpeed;
	uint8_t DS		 = SPIx_Handle->SPIConfig.DS;
	uint8_t CPHA  	 = SPIx_Handle->SPIConfig.CPHA;
	uint8_t CPOL	 = SPIx_Handle->SPIConfig.CPOL;
    uint8_t SSM      = SPIx_Handle->SPIConfig.SSM;
    uint8_t SSOE     = SPIx_Handle->SPIConfig.SSOE;


    // Clock speed
    SPIx_Handle->SPIx->CR1 &= 0xFF87; // Clear BR[5:3] bits
	SPIx_Handle->SPIx->CR1 |= (BR << SPI_CR1_BR);


	// CPOL
	SPIx_Handle->SPIx->CR1 |= (CPOL << SPI_CR1_CPOL);


	// CPHA
	SPIx_Handle->SPIx->CR1 |= (CPHA << SPI_CR1_CPHA);


	// Bus config
	if (BIDIMODE == SPI_BUS_CONFIG_FD)
    {
		SPIx_Handle->SPIx->CR1 &= ~(1 << SPI_CR1_BIDMODE); // 2-line unidirectional
    }
	else if (BIDIMODE == SPI_BUS_CONFIG_HD)
    {
		SPIx_Handle->SPIx->CR1 |= (1 << SPI_CR1_BIDMODE);  // 1-line bidirectional
    }
	else if (BIDIMODE == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// 1 line unidirectional (Rx only)
		SPIx_Handle->SPIx->CR1 |= (1 << SPI_CR1_RXONLY);
	}
    else
    {
        return false;
    }


    // Configure SSM
    if (SSM)
        SPIx_Handle->SPIx->CR1 |= (SSM << SPI_CR1_SSM);
    else
        SPIx_Handle->SPIx->CR1 &= ~(SSM << SPI_CR1_SSM);


    // Configure SSOE
    if (SSOE)
        SPIx_Handle->SPIx->CR2 |= (SSOE << SPI_CR2_SSOE);
    else
        SPIx_Handle->SPIx->CR2 &= ~(SSOE << SPI_CR2_SSOE);


    // Configure device mode MSTR bit
    SPIx_Handle->SPIx->CR1 |= (MSTR << SPI_CR1_MSTR);


	// Data frame format
    SPIx_Handle->SPIx->CR2 &= 0xF7FF; // Reset DS[8:3] bits
	SPIx_Handle->SPIx->CR2 |= (DS << SPI_CR2_DS);


    return true;
}


/**
 * @brief SPI enable/disable function
 * 
 * @param SPIx SPI register type pointer
 * @param state ENABLE or DISABLE
 */
void SPI_PeripheralControl(SPI_Reg_t *SPIx, State state)
{
    if (state == ENABLE)
    {
        SPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else if (state == DISABLE)
    {
        while (SPI_BUSY_FLAG(SPIx)); //* Wait for busy flag to be cleared

        SPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}


/**
 * @brief Disable SPI clock (APB) and disable in control register.
 * 
 * @param SPIx SPIx register type (stm32f0xx_spi.h)
 */
void SPI_DeInit(SPI_Reg_t *SPIx)
{
    //? Reset config registers

    // Disable peripheral
    SPI_PeripheralControl(SPIx, DISABLE);

    // Disable peripheral clock 
    SPI_PCLK_Control(SPIx, DISABLE);
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
 * @warning Blocks until data sent.
 * 
 * @param SPIx SPIx register type ptr
 * @param txBuffer Data array to send
 * @param dataLen Length of data buffer
 */
void SPI_sendData(SPI_Reg_t *SPIx, uint8_t *txBuffer, uint32_t dataLen)
{
    //! Block until TXE set (TX)
    while(!SPI_GetFlagStatus(SPIx, SPI_SR_TXE));


    // Check data frame format
    uint8_t DFF = (SPIx->CR2 >> 8) & 0x0FU;


    //* Load all data from txBuffer into SPI DR
    while (dataLen > 0)
    {
        //* Check TX buffer
        // Wait until TXE == 1 (TX buffer is empty)
//        while(SPI_GetFlagStatus(SPIx, SPI_TXE_FLAG) == NOT_EMPTY);

        if (DFF == SPI_DFF_8BITS) // load 8-bit data
        {
            // Load data register with data
            SPIx->DR = *txBuffer;

            // decrement data length counter
            dataLen--;

            // increment pointer in data buffer
            txBuffer++;
        }
        else if (DFF == SPI_DFF_16BITS)// Load 16-bit data
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



    // Check data frame format
    uint8_t DFF = (SPIx->CR2 >> SPI_CR2_DS) & 0x0FU;



    while (dataLen > 0)
    {
        //* Check RX buffer
        // Wait until RXNE == 1 (Rx buffer not empty)
        while(SPI_RXNE_FLAG(SPIx));

        if (DFF == SPI_DFF_8BITS) // 8-bit DFF
        {
            // Load byte into rxBuffer
            *rxBuffer = (uint8_t) SPIx->DR;

            // Increment rxBuffer ptr
            rxBuffer++;

            // Decrement dataLen counter
            dataLen--;
        }
        else if (DFF == SPI_DFF_16BITS) // 16-bit DFF
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
void SPI_sendDataIT(SPI_Handle_t *SPIx_Handle, uint8_t *txBuffer_, uint32_t txlen_)
{

    if (SPIx_Handle->txState != SPI_BUSY_IN_TX)
    {
        SPIx_Handle->txBuffer = txBuffer_;
        
        SPIx_Handle->txLen = txlen_;

        SPIx_Handle->txState = SPI_BUSY_IN_TX;

        //* Enable SPI_CR2_TXEIE control bit
        // generates interrupt when TXE set
        SPIx_Handle->SPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }
}


/**
 * @brief 
 * 
 * @param SPIx_Handle 
 * @param rxBuffer_ 
 * @param rxLen_ 
 */
void SPI_readDataIT(SPI_Handle_t *SPIx_Handle, uint8_t *rxBuffer_, uint32_t rxLen_)
{
    SPIx_Handle->rxBuffer = rxBuffer_;

    SPIx_Handle->rxLen = rxLen_;

    SPIx_Handle->rxState = SPI_BUSY_IN_RX;

    //* Enable SPI_CR2_RXEIE control bit
    // RxFIFO threshold
    // Get data frame format
    uint8_t DFF = (SPIx_Handle->SPIConfig.DS >> SPI_CR2_DS) & 0x0FU; 
    if (DFF == SPI_DFF_8BITS)
        SPIx_Handle->SPIx->CR2 |= (1 << SPI_CR2_FXRTH);
    else if (DFF == SPI_DFF_16BITS)
        SPIx_Handle->SPIx->CR2 &= ~(1 << SPI_CR2_FXRTH);

    // generates interrupt when RXNE set
    SPIx_Handle->SPIx->CR2 |= (1 << SPI_CR2_RXEIE);
}


/**
 * @brief 
 * 
 * @param SPIx_Handle 
 * @param state 
 * @param priority 
 */
void SPI_IRQInterruptConfig(SPI_Handle_t *SPIx_Handle, State state, uint8_t priority)
{
    // Determine IRQ number based on I2Cx
    uint8_t irq_num;
    if (SPIx_Handle->SPIx == SPI1)
        irq_num = SPI1_IRQ_NUM;
    else if (SPIx_Handle->SPIx == I2C2)
        irq_num = SPI2_IRQ_NUM;

    
    if (state == DISABLE)
    {
        *NVIC_ICER |= (1 << irq_num);

        SPIx_Handle->SPIx->CR2 &= 0x771F; // Clear interrupt enable bits

        return;
    }
    else if (state == ENABLE)
    {
        // Enable all interupts
        SPIx_Handle->SPIx->CR1 |= (1 << SPI_CR2_ERREIE);
        SPIx_Handle->SPIx->CR1 |= (1 << SPI_CR2_RXEIE);
        SPIx_Handle->SPIx->CR1 |= (1 << SPI_CR2_TXEIE);


        // Enable corresponding interrupt in ISER/ICER
        *NVIC_ISER |= (1 << irq_num);


        // Configure interrupt priority
        uint8_t iprNum = irq_num / 4;
        uint8_t byteOffset = irq_num % 4;

        *((uint8_t *)NVIC_IPR0 + (iprNum * 4)) |= ( priority << ((8 * byteOffset) + 6) );
    }

}


/**
 * @brief 
 * 
 * @param SPIx_Handle 
 */
void SPI_IRQHandler(SPI_Handle_t *SPIx_Handle)
{
    SPI_Reg_t *SPIx_ = SPIx_Handle->SPIx;


    //* TX interrupt
    if (SPI_TXE_FLAG(SPIx_))
    {
        SPI_sendData(SPIx_Handle->SPIx, SPIx_Handle->txBuffer, SPIx_Handle->txLen);

        if (SPIx_Handle->txLen <= 0)
        {
            if (SPIx_Handle->SPIx->CR2 & (1 << SPI_CR2_TXEIE))
                SPIx_Handle->SPIx->CR2 &= ~(1 << SPI_CR2_TXEIE); // Clear SPI_CR2_TXEIE bit after txBuffer sent
            
            SPIx_Handle->txState = SPI_READY;
        }

    }         
    

    //* RX Interrupt
    else if (SPI_RXNE_FLAG(SPIx_))   
    {
        SPI_readData(SPIx_Handle->SPIx, SPIx_Handle->rxBuffer, SPIx_Handle->rxLen);

        if (SPIx_Handle->rxLen <= 0)
        {
            if (SPIx_Handle->SPIx->CR2 & (1 << SPI_CR2_RXEIE))
                SPIx_Handle->SPIx->CR2 &= ~(1 << SPI_CR2_TXEIE); // Clear SPI_CR2_TXEIE bit after txBuffer sent
            
            SPIx_Handle->rxState = SPI_READY;

            SPI_ApplicationEventCallback(SPIx_Handle, SPI_EVENT_RX_CMPLT);
        }
    }


    //! Error interrupts
    else if ((SPIx_->SR) & (1 << SPI_SR_OVR))      
        SPI_ApplicationEventCallback(SPIx_Handle, SPI_EVENT_OVR_ERR);

    else if ((SPIx_->SR) & (1 << SPI_SR_CRCERR))
        SPI_ApplicationEventCallback(SPIx_Handle, SPI_EVENT_CRC_ERR);

    else if ((SPIx_->SR) & (1 << SPI_SR_FRE))
        SPI_ApplicationEventCallback(SPIx_Handle, SPI_EVENT_FRE_ERR);

    else if ((SPIx_->SR) & (1 << SPI_SR_MODF))
        SPI_ApplicationEventCallback(SPIx_Handle, SPI_EVENT_MODF_ERR);

}


/**
 * @brief Clears any status flags.
 * 
 * @note __weak implementation; overrided by user.
 * 
 * @param SPIx_Handle SPI handle type (stm32f0xx_spi.h)
 * @param AppEv SPI application event type (stm32f0xx_spi.h)
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *SPIx_Handle, SPI_AppEvent_t AppEv)
{

    switch (AppEv)
    {
        case SPI_EVENT_CRC_ERR: // CRC 
            // TODO: Handle CRC error
            // Clear CRCERR flag
            SPIx_Handle->SPIx->SR &= ~(1 << SPI_SR_CRCERR);
            break;
        

        case SPI_EVENT_OVR_ERR:
        {
            // OVR bit cleared when reading data register, then status register
            uint32_t DR = SPIx_Handle->SPIx->DR;
            uint32_t SR = SPIx_Handle->SPIx->SR;

            break;
        }


        case SPI_EVENT_FRE_ERR:
            SPI_PeripheralControl(SPIx_Handle->SPIx, DISABLE);
            break;

        case SPI_EVENT_MODF_ERR:
        {
            // SPE bit is cleared; SPI disabled.
            // MSTR bit is cleared.

            // Clear MODF bit:
            // Make a read access to the status register
            uint8_t SR = SPIx_Handle->SPIx->SR;
            // Write to CR1 register (re-set MSTR)
            SPIx_Handle->SPIx->CR1 |= (1 << SPI_CR1_MSTR);
            break;
        }
    }

}