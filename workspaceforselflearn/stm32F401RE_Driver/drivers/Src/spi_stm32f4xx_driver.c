/*
 * spi_stm32f4xx_driver.c
 *
 *  Created on: Jul 22, 2024
 *      Author: thanh
 */
#include "spi_stm32f4xx_driver.h"
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//weak function
}
void SPI_CloseTranmission(SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pHandle->PTxBuffer = NULL;
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pHandle->PRxBuffer = NULL;
	pHandle->RxLen =0;
	pHandle->RxState = SPI_READY;
}
static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle){
	if(pHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{	//16 bits
			pHandle->pSPIx->DR = *((uint16_t*)pHandle->PTxBuffer);
			pHandle->TxLen--;
			pHandle->TxLen--;
			(uint16_t*)pHandle->PTxBuffer++;
		}else
		{
			pHandle->pSPIx->DR = *pHandle->PTxBuffer;
			pHandle->TxLen--;
			pHandle->PTxBuffer++;
		}
	if(!pHandle->TxLen){
		//txlen zero so close spi transmission and  inform the app that tx is over
		// prevent it from setting up of TXE flag
		SPI_CloseTranmission(pHandle);
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle){
	if(pHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{	//16 bits
			*((uint16_t*)pHandle->PRxBuffer) =(uint16_t)pHandle->pSPIx->DR ;
			pHandle->RxLen--;
			pHandle->RxLen--;
			pHandle->PRxBuffer--;
			pHandle->PRxBuffer--;
		}else
		{
			*(pHandle->PRxBuffer)=(uint8_t)pHandle->pSPIx->DR ;
			pHandle->RxLen--;
			pHandle->PRxBuffer--;
		}
	if(!pHandle->RxLen){
			//txlen zero so close spi transmission and  inform the app that tx is over
			// prevent it from setting up of TXE flag
			SPI_CloseReception(pHandle);
			SPI_ApplicationEventCallback(pHandle,SPI_EVENT_RX_CMPLT);
		}
}


static void spi_ovr_interrupt_handle(SPI_Handle_t *pHandle){
	uint8_t temp;
	if(pHandle->TxState != SPI_BUSY_IN_TX){
		temp = pHandle->pSPIx->DR;
		temp = pHandle->pSPIx->SR;

	}
	(void)temp;
	SPI_ApplicationEventCallback(pHandle,SPI_EVENT_OVR_ERR);
}
void SPI_ClearOVRFlag(SPI_Handle_t *pHandle)
{ uint8_t temp;
	temp = pHandle->pSPIx->DR;
	temp = pHandle->pSPIx->SR;
  (void)temp;
}
uint8_t  SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flagname){

	if(pSPIx->SR & Flagname)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;


}


void SPI_PeriCLockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}

		}
	else
		{
			//TODO
		}

}


//Init
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//CONFIGURE THE CR1 REG
	uint32_t tempreg = 0;
	//configure the device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;
	//config the BusConfig
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{ //clear the bidi mode
		tempreg &= ~(1 <<SPI_CR1_BIDIMODE)	;
	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{ //set the bidi mode
		tempreg |= (1<< SPI_CR1_BIDIMODE )	;
	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{ //clear the bidi mode
		tempreg &= ~ (1 <<SPI_CR1_BIDIMODE);
	  //set the RXonly
		tempreg |= (1 <<SPI_CR1_RXONLY	);
	}
	// 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;


}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

// data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
while(Len>0)
{	//wait until the TXE is set
	while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
	//check the DFF bits
	if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{	//16 bits
		pSPIx->DR = *((uint16_t*)pTxBuffer);
		Len--;
		Len--;
		(uint16_t*)pTxBuffer++;
	}else
	{
		pSPIx->DR = *pTxBuffer;
		Len--;
		pTxBuffer++;
	}

}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{
while(Len>0)
{	//wait until the RXNE is set
	while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
	//check the DFF bits
	if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{	//16 bits
		pSPIx->DR = *((uint16_t*)pRxBuffer);
		Len--;
		Len--;
		(uint16_t*)pRxBuffer++;
	}else
	{
		pSPIx->DR = *pRxBuffer;
		Len--;
		pRxBuffer++;
	}

}

}
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{	//1 SAVE THE TX BUFFER ADDRESS and len info to some gloabl var
	uint8_t state = pSPIHandle->TxState ;
	if (state != SPI_BUSY_IN_TX)
	{
	pSPIHandle->PTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;

	//2. mark the state as busy so that no ohter code can take over the perepheral until the tramission is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//Enable the TXEIE control bit to get it every time TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	//data tramission will be handle by ISR code
	}
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->RxState ;
	if (state != SPI_BUSY_IN_RX)
	{
	pSPIHandle->PRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;

	//2. mark the state as busy so that no ohter code can take over the perepheral until the tramission is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;

	//Enable the TXEIE control bit to get it every time TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
	//data tramission will be handle by ISR code
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}


}
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}


}
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}


}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi )
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 6 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
			}
		}


}
void SPI_IRQPriorty(uint8_t IRQNumber , uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section  = IRQNumber %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

		*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
uint8_t temp1, temp2;
temp1 = pHandle->pSPIx->SR &(1<<SPI_SR_TXE);
temp2 = pHandle->pSPIx->CR2 &(1<<SPI_CR2_TXEIE);

if(temp1&&temp2)
{
	//handle TXE
	spi_txe_interrupt_handle(pHandle);
}
temp1 = pHandle->pSPIx->SR &(1<<SPI_SR_RXNE);
temp2 = pHandle->pSPIx->CR2 &(1<<SPI_CR2_RXNEIE);

if(temp1&&temp2)
{
	//handle RXE
	spi_rxne_interrupt_handle(pHandle);
}
temp1 = pHandle->pSPIx->SR &(1<<SPI_SR_OVR);
temp2 = pHandle->pSPIx->CR2 &(1<<SPI_CR2_ERRIE);

if(temp1&&temp2)
{
	//handle OVR
	spi_ovr_interrupt_handle(pHandle);
}


}


