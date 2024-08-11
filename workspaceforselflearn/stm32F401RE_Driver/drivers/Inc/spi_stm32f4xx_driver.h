/*
 * spi_stm32f3xx_driver.h
 *
 *  Created on: Jul 22, 2024
 *      Author: thanh
 */

#ifndef INC_SPI_STM32F4XX_DRIVER_H_
#define INC_SPI_STM32F4XX_DRIVER_H_

#include "stm32F401RE.h"



typedef struct
{	uint8_t SPI_DeviceMode; //MASTER OR SLAVE
	uint8_t SPI_BusConfig; // HALF OR FULL DUPLEX
	uint8_t	SPI_SclkSpeed;
	uint8_t SPI_DFF; //DATA SIZE
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle struc for SPI peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t *PTxBuffer;
	uint8_t	*PRxBuffer;
	uint8_t TxLen;
	uint8_t	RxLen;
	uint8_t TxState;
	uint8_t	RxState;
}SPI_Handle_t;



static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pHandle);
//Device mode
#define SPI_DEVICE_MODE_MASTER   1
#define SPI_DEVICE_MODE_SLAVE	 0

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3
 //SCLK SPEED
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

#define SPI_SSM_HW	1
#define SPI_SSM_SW	0

#define SPI_TXE_FLAG	( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)

#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4
// Peripheral clock
void SPI_PeriCLockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


//Init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
// data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len);



void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi );
void SPI_IRQPriorty(uint8_t IRQNumber , uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_clearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTranmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);




#endif /* INC_SPI_STM32F4XX_DRIVER_H_ */
