/*
 * i2c_stm32f4xx_driver.c
 * Author: thanh
 * Notes:
 *  - Organized into sections (Includes/Defs -> RCC helpers -> I2C helpers -> Public API)
 *  - Fixed typos (ABP->APB, RXE->RXNE, function name mismatches)
 *  - Added I2C_ClearADDRFlag() sequence (read SR1 then SR2)
 *  - Made MasterSend/MasterReceive blocking & simple (no repeated start)
 */

#include "i2c_stm32f4xx_driver.h"

/*==============================*
 *      Local Definitions       *
 *==============================*/

/* If these bit positions/macros aren’t in your header, uncomment them here */
// #define I2C_CR1_PE      0
// #define I2C_CR1_START   8
// #define I2C_CR1_STOP    9
// #define I2C_CR1_ACK     10

/* Flag helpers expected in your header:
 * I2C_FLAG_SB, I2C_FLAG_ADDR, I2C_FLAG_TXE, I2C_FLAG_BTF, I2C_FLAG_RXNE
 * Get them from RM0090 or your existing header.
 */

static const uint16_t AHB_PRESC_TABLE[8]  = {2,4,8,16,64,128,256,512};
static const uint16_t APB_PRESC_TABLE[4]  = {2,4,8,16};

/*==============================*
 *         Declarations         *
 *==============================*/
static void     I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void     I2C_GenerateStopCondition (I2C_RegDef_t *pI2Cx);
static void     I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void     I2C_ExecuteAddressPhaseRead (I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t * pI2CHandle);
static uint32_t RCC_GetPLLOutputClock(void); /* TODO if you use PLL as SYSCLK */
static uint32_t RCC_GetPCLK1Value(void);

/* Public (from header) */
void            I2C_PeriCLockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void            I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t         I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);

/*==============================*
 *        RCC Clock Math        *
 *==============================*/

static uint32_t RCC_GetPLLOutputClock(void)
{
    /* Implement if SYSCLK is PLL. For now return 0 to signal “unused”. */
    return 0U;
}

static uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t sysclk;
    uint32_t pclk1;
    uint8_t  clksrc  = (uint8_t)((RCC->CFGR >> 2) & 0x3);   // SWS bits
    uint8_t  hpre    = (uint8_t)((RCC->CFGR >> 4) & 0xF);   // HPRE
    uint8_t  ppre1   = (uint8_t)((RCC->CFGR >> 10) & 0x7);  // PPRE1

    /* SYSCLK source decode */
    switch (clksrc) {
        case 0: sysclk = 16000000U; break;                  // HSI
        case 1: sysclk = 8000000U;  break;                  // HSE (if 8 MHz)
        case 2: sysclk = RCC_GetPLLOutputClock(); break;    // PLL
        default: sysclk = 16000000U; break;
    }

    /* AHB prescaler */
    uint32_t ahb_div = 1U;
    if (hpre >= 8) {
        ahb_div = AHB_PRESC_TABLE[hpre - 8];
    }

    /* APB1 prescaler */
    uint32_t apb1_div = 1U;
    if (ppre1 >= 4) {
        apb1_div = APB_PRESC_TABLE[ppre1 - 4];
    }

    pclk1 = (sysclk / ahb_div) / apb1_div;
    return pclk1;
}

/*==============================*
 *        I2C Low-Level         *
 *==============================*/

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx)
{
    pI2Cx->CR1 |= (1U << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1U << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    uint8_t addr = (uint8_t)(SlaveAddr << 1);
    addr &= (uint8_t)~(1U);  // R/W = 0 for write
    pI2Cx->DR = addr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    uint8_t addr = (uint8_t)(SlaveAddr << 1);
    addr |= 1U;              // R/W = 1 for read
    pI2Cx->DR = addr;
}

/* Clearing ADDR requires reading SR1 then SR2 (see RM0090) */
static void I2C_ClearADDRFlag(I2C_Handle_t * pI2CHandle)
{
    volatile uint32_t dummy;
    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)){
    	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
    		if(pI2CHandle->RxSize == 1){
    			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);


    		}
    	}
    	else{
    		dummy = pI2CHandle->pI2Cx->SR1;
			(void)dummy;
			dummy = pI2CHandle->pI2Cx->SR2;
			(void)dummy;
    	}
    }
    else{
    	dummy = pI2CHandle->pI2Cx->SR1;
		(void)dummy;
		dummy = pI2CHandle->pI2Cx->SR2;
		(void)dummy;
    }

}

void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->RxSize == 1){
						//load the date in to DR
						pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
						pI2CHandle->RxLen--;
					}
					if(pI2CHandle->RxSize > 1){
						if(pI2CHandle->RxLen == 2){
							I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

						}
							//read DR
						*pI2CHandle->pRxBuffer++ = (uint8_t)pI2CHandle->pI2Cx->DR;
						pI2CHandle->pRxBuffer++;
						pI2CHandle->RxLen--;
					}
					if(pI2CHandle->RxLen == 0){
						if (pI2CHandle->Sr == I2C_DISABLE_SR)
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
						I2C_CloseRxData();
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

					}
}
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}

/*==============================*
 *          Public API          *
 *==============================*/

void I2C_PeriCLockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        if (pI2Cx == I2C1)      { I2C1_PCLK_EN(); }
        else if (pI2Cx == I2C2) { I2C2_PCLK_EN(); }
        else if (pI2Cx == I2C3) { I2C3_PCLK_EN(); }
    } else {
        /* TODO: implement disable paths if you need them */
    }
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == I2C_ACK_ENABLE) {
        pI2Cx->CR1 |=  (1U << I2C_CR1_ACK);
    } else {
        pI2Cx->CR1 &= ~(1U << I2C_CR1_ACK);
    }
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
    return (pI2Cx->SR1 & FlagName) ? FLAG_SET : FLAG_RESET;
}

/* Blocking init for SM (≤100kHz) or FM (400kHz) */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;

    /* 1) ACK control */
    tempreg = pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
    pI2CHandle->pI2Cx->CR1 = tempreg;

    /* 2) CR2 FREQ field (PCLK1 in MHz, max 42 MHz on F4) */
    uint32_t pclk1 = RCC_GetPCLK1Value();
    pI2CHandle->pI2Cx->CR2 = (pclk1 / 1000000U) & 0x3F;

    /* 3) Own address (7-bit) */
    tempreg  = ((uint32_t)pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
    tempreg |= (1U << 14);                      // Must be 1 per RM
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    /* 4) CCR for speed */
    tempreg = 0;
    uint32_t speed = pI2CHandle->I2C_Config.I2C_SCLSpeed;
    if (speed <= I2C_SCL_SPEED_SM) {
        /* Standard mode: CCR = PCLK1 / (2*SCL) */
        uint32_t ccr = pclk1 / (2U * speed);
        if (ccr < 4U) ccr = 4U;                 // simple guard
        tempreg = (ccr & 0xFFFU);
    } else {
        /* Fast mode */
        tempreg |= (1U << 15);                  // F/S = 1
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
        uint32_t ccr;
        if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
            /* DUTY=0 -> Tlow/Thigh = 2 */
            ccr = pclk1 / (3U * speed);
        } else {
            /* DUTY=1 -> 16/9 */
            ccr = pclk1 / (25U * speed);
        }
        if (ccr == 0U) ccr = 1U;
        tempreg |= (ccr & 0xFFFU);
    }
    pI2CHandle->pI2Cx->CCR = tempreg;

    /* 5) TRISE (max rise time in terms of PCLK1 cycles) */
    if (speed <= I2C_SCL_SPEED_SM) {
        /* 1000ns max rise -> TRISE = Freq_MHz + 1 */
        pI2CHandle->pI2Cx->TRISE = ((pclk1 / 1000000U) + 1U) & 0x3F;
    } else {
        /* 300ns max rise -> TRISE = (Freq_MHz * 300ns) + 1 */
        uint32_t trise = ((pclk1 / 1000000U) * 300U) / 1000U + 1U;
        pI2CHandle->pI2Cx->TRISE = trise & 0x3F;
    }

    /* 6) Enable peripheral */
    pI2CHandle->pI2Cx->CR1 |= (1U << I2C_CR1_PE);
}

/*==============================*
 *   Master Blocking Transfers  *
 *==============================*/

/* Master WRITE: send Len bytes from pTxBuffer to SlaveAddr (7-bit). */
void I2C_MasterSendData(I2C_Handle_t *h, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
    /* START */
    I2C_GenerateStartCondition(h->pI2Cx);
    while (!I2C_GetFlagStatus(h->pI2Cx, I2C_FLAG_SB));

    /* Address (write) */
    I2C_ExecuteAddressPhaseWrite(h->pI2Cx, SlaveAddr);
    while (!I2C_GetFlagStatus(h->pI2Cx, I2C_FLAG_ADDR));
    I2C_ClearADDRFlag(h->pI2Cx);  // clears ADDR

    /* Send bytes */
    while (Len > 0) {
        while (!I2C_GetFlagStatus(h->pI2Cx, I2C_FLAG_TXE));
        h->pI2Cx->DR = *pTxBuffer++;
        Len--;
    }

    /* Wait for TXE=1 and BTF=1 (data shifted out and byte transfer finished) */
    while (!I2C_GetFlagStatus(h->pI2Cx, I2C_FLAG_TXE));
    while (!I2C_GetFlagStatus(h->pI2Cx, I2C_FLAG_BTF));

    /* STOP */
    I2C_GenerateStopCondition(h->pI2Cx);
}

/* Master READ: receive Len bytes into pRxBuffer from SlaveAddr (7-bit). */
void I2C_MasterReceiveData(I2C_Handle_t *h, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
    /* Enable ACK if length > 1 (we’ll manage it) */
    if (Len > 1) I2C_ManageAcking(h->pI2Cx, I2C_ACK_ENABLE);

    /* START */
    I2C_GenerateStartCondition(h->pI2Cx);
    while (!I2C_GetFlagStatus(h->pI2Cx, I2C_FLAG_SB));

    /* Address (read) */
    I2C_ExecuteAddressPhaseRead(h->pI2Cx, SlaveAddr);
    while (!I2C_GetFlagStatus(h->pI2Cx, I2C_FLAG_ADDR));

    if (Len == 1) {
        /* For single byte, NACK then clear ADDR, then read and STOP */
        I2C_ManageAcking(h->pI2Cx, I2C_ACK_DISABLE);
        I2C_ClearADDRFlag(h->pI2Cx);
        while (!I2C_GetFlagStatus(h->pI2Cx, I2C_FLAG_RXNE));
        I2C_GenerateStopCondition(h->pI2Cx);
        *pRxBuffer = (uint8_t)h->pI2Cx->DR;
    } else {
        /* Multi-byte: clear ADDR, then read until last 2 bytes */
        I2C_ClearADDRFlag(h->pI2Cx);

        for (uint32_t i = Len; i > 0; i--) {
            while (!I2C_GetFlagStatus(h->pI2Cx, I2C_FLAG_RXNE));

            if (i == 2) {
                /* Prepare NACK for the last byte and issue STOP at the edge of last-2 */
                I2C_ManageAcking(h->pI2Cx, I2C_ACK_DISABLE);
                if(h->Sr == I2C_DISABLE_SR)
                	I2C_GenerateStopCondition(h->pI2Cx);
            }

            *pRxBuffer++ = (uint8_t)h->pI2Cx->DR;
            pRxBuffer++;
        }
    }

    /* Re-enable ACK if globally configured */
    if (h->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(h->pI2Cx, I2C_ACK_ENABLE);
    }
}
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pTxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint8_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_SR1_SB);

	if( temp1 && temp3){
		//SB evenet
		//Interrupt is started because of SB event
		// execute the address phase
		// Base on the application state
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);

		}
	}
	temp3 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_SR1_ADDR);
	if( temp1 && temp3){
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}
	temp3 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_SR1_BTF);
	if( temp1 && temp3){
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			if(pI2CHandle->pI2Cx->CR2 & (1 << I2C_SR1_TXE)){
				//BTF and TXE is 1
				// 1 generate stop condition
				if(pI2CHandle->Sr = I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				// reset the member
				I2C_CloseSendData();
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);

			}

		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			;
		}


	}
	temp3 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_SR1_STOPF);
	if( temp1 && temp3){
		//stopF flag is ste
		//clea the stopf( 1 read Sr1 and write to cr1
		pI2CHandle ->pI2Cx->CR1 |= 0x0000;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}

	temp3 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_SR1_TXE);
	//handle the event generated by tx event
	if( temp1 && temp2 && temp3){
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_MasterHandleTXEInterrupt(pI2CHandle );
		}
	}

	temp3 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_SR1_RXNE);
		//handle the event generated by Rx event
		if( temp1 && temp2 && temp3){
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}


}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t temp1,temp2;

	    //Know the status of  ITERREN control bit in the CR2
		temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
		if(temp1  && temp2 )
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

			//Implement the code to notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
		}

	/***********************Check for arbitration lost error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
		if(temp1  && temp2)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
			//Implement the code to notify the application about the error
			 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
		}

	/***********************Check for ACK failure  error************************************/

		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
		if(temp1  && temp2)
		{
			//This is ACK failure error

		    //Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
			//Implement the code to notify the application about the error
			 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
		}

	/***********************Check for Overrun/underrun error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
		if(temp1  && temp2)
		{
			//This is Overrun/underrun

		    //Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
			//Implement the code to notify the application about the error
			 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
		}

	/***********************Check for Time out error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
		if(temp1  && temp2)
		{
			//This is Time out error

		    //Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
			//Implement the code to notify the application about the error
			 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
		}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
    return (uint8_t) pI2C->DR;
}


