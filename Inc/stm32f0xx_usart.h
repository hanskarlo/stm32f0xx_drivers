/*
 * STM32F4x_gpio_driver.h
 *
 *  Created on: 27-Nov-2018
 *      Author: kiran
 */

#ifndef STM32F0XX_UART_H_
#define STM32F0XX_UART_H_

#include "stm32f0xx.h"


/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint32_t baudRate;
	uint8_t numStopBits;
	uint8_t wordLen;
	uint8_t parity;
}USART_Config_t;


/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_Reg_t *USARTx;
	USART_Config_t   USART_Config;
	uint8_t *txBuffer;
	uint8_t *rxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;


#define USART_CR1_RESET_VALUE   0x00000000
#define USART_BRR_RESET_VALUE   USART_CR1_RESET_VALUE   


//* USART bit positions 

/*
 * CR1 bit positions 
*/
#define USART_CR1_M1        28
#define USART_CR1_OVER8     15
#define USART_CR1_M0        12
#define USART_CR1_TXEIE     7
#define USART_CR1_TCIE      6
#define USART_CR1_RXNEIE    5
#define USART_CR1_TE        3
#define USART_CR1_RE        2
#define USART_CR1_UE        0
#define USART_CR1_RESET_VALUE   0x00000000

/*
 * CR2 bit positions
*/
#define USART_CR2_STOP      12

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

#define DEFAULT_BAUD_RATE                   USART_STD_BAUD_9600

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3


/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


/*
 * USART flags
 */

#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0


#define 	USART_EVENT_TX_CMPLT   0
#define		USART_EVENT_RX_CMPLT   1
#define		USART_EVENT_IDLE      2
#define		USART_EVENT_CTS       3
#define		USART_EVENT_PE        4
#define		USART_ERR_FE     	5
#define		USART_ERR_NE    	 6
#define		USART_ERR_ORE    	7

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral enable/disable
*/
void USART_PeripheralControl(USART_Reg_t *USARTx, State enable);

/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_Reg_t *USARTx, State enable);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *USARTxHandle);
void USART_DeInit(USART_Handle_t *USARTxHandle);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *USARTxHandle, uint8_t *txBuffer, uint32_t dataLen);
void  USART_ReceiveData(USART_Handle_t *USARTxHandle,uint8_t *rxBuffer, uint32_t dataLen);
uint8_t USART_SendDataIT(USART_Handle_t *USARTxHandle,uint8_t *txBuffer, uint32_t dataLen);
uint8_t USART_ReceiveDataIT(USART_Handle_t *USARTxHandle,uint8_t *rxBuffer, uint32_t dataLen);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, State enable);
void USART_IRQPriorityConfig(uint8_t irqNum, uint32_t irqPriority);
void USART_IRQHandling(USART_Handle_t *USARTxHandle);

/*
 * Other Peripheral Control APIs
 */

uint8_t USART_GetFlagStatus(USART_Reg_t *USARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_Reg_t *USARTx, uint16_t StatusFlagName);
void USART_PeripheralControl(USART_Reg_t *USARTx, State enable);
void USART_SetBaudRate(USART_Reg_t *USARTx, uint32_t baud);


/*
 * Application Callbacks
 */
void USART_ApplicationEventCallback(USART_Handle_t *USARTxHandle, uint8_t appEvent);




#endif /* STM32F0XX_UART_H_ */



