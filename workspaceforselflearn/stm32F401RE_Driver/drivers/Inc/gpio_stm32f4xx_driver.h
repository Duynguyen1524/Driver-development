/*
 * gpio_stm32f4xx_driver.h
 *
 *  Created on: May 12, 2024
 *      Author: thanh
 */

#ifndef INC_GPIO_STM32F4XX_DRIVER_H_
#define INC_GPIO_STM32F4XX_DRIVER_H_

#include "stm32F401RE.h"
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;/*!< possible values from @GPIO_PIN_PU >*/
	uint8_t GPIO_PinOPType;			/*!< possible values from @GPIO_OP >*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;
//define pin number
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15
//define the possible mode of periphral
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

//OUTPUT TYPE
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1
//SPEED
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3
//PULL UP AND PULL DOWN
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


//API create
void GPIO_PeriCLockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


//read and write from data

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * IRQ config and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi );
void GPIO_IRQPriorty(uint8_t IRQNumber , uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_GPIO_STM32F4XX_DRIVER_H_ */
