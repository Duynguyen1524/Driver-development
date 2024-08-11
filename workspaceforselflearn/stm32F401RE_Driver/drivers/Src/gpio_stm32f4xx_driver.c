/*
 * gpio_stm32f4xx_driver.c
 *
 *  Created on: May 12, 2024
 *      Author: thanh
 */

#include "gpio_stm32f4xx_driver.h"

void GPIO_PeriCLockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}

}
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
		{
		temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;

		}
	else{// for interrupt
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//config the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//config the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
						//clear the RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//config both
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//config the port selection
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode<<(temp2*4);

		//enable the exti interrupt using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//config the GPIO port selection in syscfg
	}
	temp = 0;
	//config the speed of
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	//config the PUPD config
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//CONFIG THE OUTPUT TYPE
	temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//CONFIG THE ALT
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint32_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1]=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<4*temp2);
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
			if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET() ;
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET() ;
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET() ;
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET() ;
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET() ;
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET() ;
			}else if (pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET() ;
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET() ;
			}else if (pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET() ;
			}
}
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber)& 0x00000001);//shift the bit by the amount of pin number care only abou the last bit so mask every other bit
	return value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |=(1<<PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^=(1<<PinNumber);
}
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi ){
	if( EnorDi == ENABLE){
		if(IRQNumber<= 31)
		{//prgram ISER0 Reg
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)
		{//prgram ISER1 Reg
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >=64 && IRQNumber <96)
		{
			//prgram ISER2 Reg
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber<= 31)
		{//prgram ICER0 Reg
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)
		{//prgram ICER1 Reg
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >=64 && IRQNumber <96)
		{//prgram ICER2 Reg
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}

	}

}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8 * iprx_section) + (8- NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx*4)) |=  ( IRQPriority  <<  shift_amount );
}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI pr reg conresponding to the pin number
	if(EXTI->PR &(1<<PinNumber))
	{
		EXTI->PR |= (1<< PinNumber);
	}
}













