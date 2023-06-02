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
    uint8_t  deviceAddress;
    uint8_t  ackControl;
    uint8_t  fmDutyCycle;
}I2C_Config_t;

typedef struct 
{
    I2C_Reg_t       *I2Cx;
    I2C_Config_t    I2C_Config;
    uint8_t         *txBuffer;
    uint8_t         *rxBuffer;
    uint32_t        txlen;
    uint32_t        rxlen;
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


//* Register bit positions

/*
 * I2C Control Register (2) bit positions 
*/
#define I2C_CR2_ADD10   11
#define I2C_CR2_START   13
#define I2C_CR2_STOP    14

/*
 * I2C Interrupt and Status Register bit positions 
*/
#define I2C_ISR_TXE     0
#define I2C_ISR_TXIS    1
#define I2C_ISR_RXNE    2
#define I2C_ISR_ADDR    3
#define I2C_ISR_NACKF   4
#define I2C_ISR_STOPF   5   
#define I2C_ISR_TC5     6
#define I2C_ISR_TCR     7
#define I2C_ISR_BERR    8
#define I2C_ISR_ARLO    9
#define I2C_ISR_OVR     10
#define I2C_ISR_PECERR  11
#define I2C_ISR_TIMEOUT 12
#define I2C_ISR_ALERT   13
#define I2C_ISR_BUSY    15
#define I2C_ISR_DIR     16


#define I2C_FLAG_TXE        (1 << I2C_ISR_TXE)  
#define I2C_FLAG_RXNE       (1 << I2C_ISR_RXNE)
#define I2C_FLAG_START      (1 << I2C_CR2_START)
#define I2C_FLAG_OVR        (1 << I2C_ISR_OVR)
#define I2C_FLAG_AF         (1 << I2C_ISR_NACKF)
#define I2C_FLAG_ARLO       (1 << I2C_ISR_ARLO)
#define I2C_FLAG_BERR       (1 << I2C_ISR_BERR)
#define I2C_FLAG_STOPF      (1 << I2C_ISR_STOPF)
#define I2C_FLAG_ADD10      (1 << I2C_ISR_ADDR)
#define I2C_FLAG_BTF        (1 << I2C_CR2_ADD10)
#define I2C_FLAG_ADDR       (1 << I2C_ISR_ADDR)
#define I2C_FLAG_TIMEOUT    (1 << I2C_ISR_TIMEOUT)


#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET


//* I2C application events macros
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9


/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_Reg_t *I2Cx, State enable);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *I2Cx_Handle);
void I2C_DeInit(I2C_Reg_t *I2Cx);


/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *I2Cx_Handle,uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *I2Cx_Handle,uint8_t *rxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *I2Cx_Handle,uint8_t *txBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *I2Cx_Handle,uint8_t *rxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *I2Cx_Handle);
void I2C_CloseSendData(I2C_Handle_t *I2Cx_Handle);


void I2C_SlaveSendData(I2C_Reg_t *I2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_Reg_t *I2Cx);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t irqNum, State enable);
void I2C_IRQPriorityConfig(uint8_t irqNum, uint32_t irqPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *I2Cx_Handle);
void I2C_ER_IRQHandling(I2C_Handle_t *I2Cx_Handle);


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_Reg_t *I2Cx, State enable);
uint8_t I2C_GetFlagStatus(I2C_Reg_t *I2Cx , uint32_t flagName);
void I2C_ManageAcking(I2C_Reg_t *I2Cx, State enable);
void I2C_GenerateStopCondition(I2C_Reg_t *I2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_Reg_t *I2Cx,State enable);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *I2Cx_Handle, uint8_t appEvent);


#endif