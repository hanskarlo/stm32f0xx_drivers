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
    uint8_t SSOE;
}SPI_Config_t;


/*
 *Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_Reg_t 		*SPIx;      // This holds the base address of SPIx(x = 1,2) peripheral
	SPI_Config_t 	SPIConfig;  // SPI config
	uint8_t 		*txBuffer;  // To store the app. Tx buffer address
	uint8_t 		*rxBuffer;	// To store the app. Rx buffer address
	uint32_t 		txLen;		// To store Tx len
	uint32_t 		rxLen;		// To store Rx len
	uint8_t 		txState;	// To store Tx state
	uint8_t 		rxState;	// To store Rx state
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
typedef enum
{
    SPI_EVENT_TX_CMPLT,   
    SPI_EVENT_RX_CMPLT,   
    SPI_EVENT_OVR_ERR,    
    SPI_EVENT_MODF_ERR,
    SPI_EVENT_CRC_ERR,    
    SPI_EVENT_FRE_ERR
}SPI_AppEvent_t;





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
#define SPI_CR1_BIDMODE     15
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_CRCEN       13
#define SPI_CR1_CRCNEXT     12
#define SPI_CR1_CRCL        11
#define SPI_CR1_RXONLY      10
#define SPI_CR1_SSM         9
#define SPI_CR1_SSI         8
#define SPI_CR1_LSBFIRST    7
#define SPI_CR1_SPE         6
#define SPI_CR1_BR          3
#define SPI_CR1_MSTR        2
#define SPI_CR1_CPOL        1
#define SPI_CR1_CPHA        0

/*
 * @SPI_CR2
*/
#define SPI_CR2_FXRTH   12
#define SPI_CR2_DS      8 
#define SPI_CR2_TXEIE   7
#define SPI_CR2_RXEIE   6
#define SPI_CR2_ERREIE  5
#define SPI_CR2_SSOE    2
#define SPI_CR2_TXDMAEN 1
#define SPI_CR2_RXDMAEN 0

/*
 * @SPI_SR 
*/
#define SPI_SR_FRE         8
#define SPI_SR_BSY         7
#define SPI_SR_OVR         6
#define SPI_SR_MODF        5
#define SPI_SR_CRCERR      4
#define SPI_SR_TXE         1
#define SPI_SR_RXNE        0

#define NOT_EMPTY   0
#define EMPTY       1

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG(SPIx)  ( (SPIx->SR) & (1 << SPI_SR_TXE)  )
#define SPI_RXNE_FLAG(SPIx) ( (SPIx->SR) & (1 << SPI_SR_RXNE) )          
#define SPI_BUSY_FLAG(SPIx) ( (SPIx->SR) & (1 << SPI_SR_BSY)  ) 
#define SPI_ERR(SPIx)       ( (SPIx->SR) & (1 << SPI_SR_CRCERR) ) ? 1 : \
                            ( (SPIx->SR) & (1 << SPI_SR_MODF)   ) ? 1 : \
                            ( (SPIx->SR) & (1 << SPI_SR_OVR)    ) ? 1 : \
                            ( (SPIx->SR) & (1 << SPI_SR_FRE)    ) ? 1 : 0


//* SPI Interrupt Vector Number
#define SPI1_IRQ_NUM    25
#define SPI2_IRQ_NUM    26



/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PCLK_Control(SPI_Reg_t *SPIx, State sclkState);

/*
 * Init and De-init
 */

const bool SPI_Init(SPI_Handle_t *SPIx_Handle);
void SPI_DeInit(SPI_Reg_t *SPIx);


/*
 * Data Send and Receive
 */
void SPI_sendData(SPI_Reg_t *SPIx, uint8_t *txBuffer, uint32_t dataLen);
void SPI_readData(SPI_Reg_t *SPIx, uint8_t *rxBuffer, uint32_t dataLen);

void SPI_sendDataIT(SPI_Handle_t *SPIx_Handle, uint8_t *txBuffer, uint32_t txLen);
void SPI_readDataIT(SPI_Handle_t *SPIx_Handle, uint8_t *rxBuffer, uint32_t rxLen);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(SPI_Handle_t *SPIx_Handle, State state, uint8_t priority);
void SPI_IRQHandler(SPI_Handle_t *SPIx_Handle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *SPIx_Handle, SPI_AppEvent_t AppEv);

#endif /* INC_STM32F0XX_SPI_H_ */
