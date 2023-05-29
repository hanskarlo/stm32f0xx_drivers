/*
 * stm32f0xx_spi.h
 *
 *  Created on: Mar. 22, 2023
 *      Author: hanzahar
 */

#ifndef INC_STM32F0XX_SPI_H_
#define INC_STM32F0XX_SPI_H_

#include "stm32f0xx.h"


/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t DeviceMode;
	uint8_t BusConfig;
	uint8_t SclkSpeed;
	uint8_t DS;
	uint8_t CPOL;
	uint8_t CPHA;
	uint8_t SSM;
}SPI_Config_t;


/*
 *Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_Reg_t 		*SPIx;   /*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
	SPI_Config_t 	SPIConfig;
	uint8_t 		*txBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*rxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		txLen;		/* !< To store Tx len > */
	uint32_t 		rxLen;		/* !< To store Tx len > */
	uint8_t 		txState;	/* !< To store Tx state > */
	uint8_t 		rxState;	/* !< To store Rx state > */
}SPI_Handle_t;


/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4



/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER    1
#define SPI_DEVICE_MODE_SLAVE     0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD                1
#define SPI_BUS_CONFIG_HD                2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2             	0
#define SPI_SCLK_SPEED_DIV4             	1
#define SPI_SCLK_SPEED_DIV8             	2
#define SPI_SCLK_SPEED_DIV16             	3
#define SPI_SCLK_SPEED_DIV32             	4
#define SPI_SCLK_SPEED_DIV64             	5
#define SPI_SCLK_SPEED_DIV128             	6
#define SPI_SCLK_SPEED_DIV256             	7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS 	0b00000111
#define SPI_DFF_16BITS  0b00001111

/*
 * @CPOL
 */
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN     1
#define SPI_SSM_DI     0

/*
 * @SPI_CR1
*/
#define SPI_CR1_SPE 6

/*
 * @SPI_CR2
*/
#define TXEIE   7
#define RXEIE   6

/*
 * @SPI_SR 
*/
#define FRE         8
#define BSY         7
#define OVR         6
#define MODF        5
#define CRCERR      4
#define TXE         1
#define RXNE        0

#define NOT_EMPTY   0
#define EMPTY       1

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG(SPIx)  ( (SPIx->SR) & (1 << TXE)  )
#define SPI_RXNE_FLAG(SPIx) ( (SPIx->SR) & (1 << RXNE) )          
#define SPI_BUSY_FLAG(SPIx) ( (SPIx->SR) & (1 << BSY)  ) 
#define SPI_ERR(SPIx)       ( (SPIx->SR) & (1 << CRCERR) ) ? 1 : \
                            ( (SPIx->SR) & (1 << MODF)   ) ? 1 : \
                            ( (SPIx->SR) & (1 << OVR)    ) ? 1 : \
                            ( (SPIx->SR) & (1 << FRE)    ) ? 1 : 0




/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_Reg_t *SPIx, State sclkState);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *SPIxHandle);
void SPI_DeInit(SPI_Reg_t *SPIx);


/*
 * Data Send and Receive
 */
void SPI_sendData(SPI_Reg_t *SPIx,uint8_t *txBuffer, uint32_t dataLen);
void SPI_readData(SPI_Reg_t *SPIx, uint8_t *rxBuffer, uint32_t dataLen);

void SPI_sendDataIT(SPI_Handle_t *SPIxHandle, uint8_t *txBuffer_, uint32_t txLen_);
void SPI_readDataIT(SPI_Handle_t *SPIxHandle, uint8_t *rxBuffer_, uint32_t rxLen_);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t irqNo, State state);
void SPI_IRQPriorityConfig(uint8_t irqNo, uint32_t irqPrio);
void SPI_IRQHandler(SPI_Handle_t *SPIxHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_Enable(SPI_Reg_t *SPIx, uint8_t enable);
void SPI_SSIConfig(SPI_Reg_t *SPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_Reg_t *SPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_Reg_t *SPIx , uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_Reg_t *SPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_STM32F0XX_SPI_H_ */
