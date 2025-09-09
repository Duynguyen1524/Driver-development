/*
 * i2c_stm32f4xx_driver.h
 *
 *  Created on: Aug 10, 2024
 *      Author: thanh
 */

#ifndef INC_I2C_STM32F4XX_DRIVER_H_
#define INC_I2C_STM32F4XX_DRIVER_H_

#include "stm32F401RE.h"
#define I2C_ACK_ENABLE 1
#define I2C_ACK_DISABLE 0

#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM4K 	400000
#define I2C_SCL_SPEED_FM2K  200000


/*
 * @I2C_AckControl
 */



/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2        0
#define I2C_FM_DUTY_16_9     1



#define I2C_FLAG_TXE   		( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

#define I2C_EV_TX_CMPLT 	0
#define I2C_EV_RX_CMPLT 	1
#define I2C_EV_STOP 		2

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7
typedef struct {
    uint32_t I2C_SCLSpeed;
    uint8_t  I2C_DeviceAddress; //mention by using
    uint8_t  I2C_ACKControl;
    uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct {
    I2C_RegDef_t *pI2Cx;       // Pointer to I2C peripheral base address
    I2C_Config_t I2C_Config;   // I2C configuration settings
    uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
	uint32_t        RxSize;		/* !< To store Rx size  > */
	uint8_t         Sr;			/* !< To store repeated start value  > */
} I2C_Handle_t;

// Peripheral clock
void I2C_PeriCLockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


//Init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t Slaveaddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t Slaveaddr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi );
void I2C_IRQPriorty(uint8_t IRQNumber , uint32_t IRQPriority);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);


#endif /* INC_I2C_STM32F4XX_DRIVER_H_ */
