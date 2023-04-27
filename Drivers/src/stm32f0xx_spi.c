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



void SPI_DeInit(SPI_Reg_t *SPIx)
{

}






void SPI_SendData(SPI_Reg_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{

}
