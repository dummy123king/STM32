/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Feb 20, 2024
 *      Author: mirafra
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"

/**
 *
 * GPIO pin MODES
 */
#define GPIO_MODE_IN		0	// 	Input (reset state)
#define GPIO_MODE_OUT		1	// 	General purpose output mode
#define GPIO_MODE_ALTFN		2	// 	Alternate function mode
#define GPIO_MODE_ANALOG	3	// 	Analog mode
#define GPIO_MODE_IT_FT		4	// 	Input at FALLING EDGE
#define GPIO_MODE_IT_RT		5	// 	Input at RISING EDGE
#define GPIO_MODE_IT_FRT	6	// 	Input at RISING and FALLING EDGE

/*
 * @GPIO_PIN_OP_TYPES
 * GPIO Pin OUTPUT Types
 * */
#define GPIO_OP_TYPE_PP 		0	//	Output push-pull (reset state)
#define GPIO_OP_TYPE_OD			1	//	Output open-drain


/*
 * @GPIO_PIN_OUT_SPEEDS
 * GPIO Pin OUTPUT SPEED MODES
 * */
#define GPIO_OP_SPEED_LOW			0	//	Low speed
#define GPIO_OP_SPEED_MEDIUM		1	//	Medium speed
#define GPIO_OP_SPEED_FAST			2	//	FAST speed
#define GPIO_OP_SPEED_HIGH			3	//	Very High Speed


/*
 * @GPIO_PIN_PP_PD
 * GPIO Pin pull-Up and Pull Down
 *  */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/*
 * GPIO_PIN_NUMBERS
 */
enum GPIO_PINS{
	GPIO_PIN_NO_0,
	GPIO_PIN_NO_1,
	GPIO_PIN_NO_2,
	GPIO_PIN_NO_3,
	GPIO_PIN_NO_4,
	GPIO_PIN_NO_5,
	GPIO_PIN_NO_6,
	GPIO_PIN_NO_7,
	GPIO_PIN_NO_8,
	GPIO_PIN_NO_9,
	GPIO_PIN_NO_10,
	GPIO_PIN_NO_11,
	GPIO_PIN_NO_12,
	GPIO_PIN_NO_13,
	GPIO_PIN_NO_14,
	GPIO_PIN_NO_15
};


typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;				//	@GPIO pin MODES
	uint8_t GPIO_PinSpeed;				//	@GPIO_PIN_OUT_SPEEDS
	uint8_t GPIO_PinPuPdCtrl;			//	@GPIO_PIN_PP_PD
	uint8_t GPIO_PinOPType;				//	@GPIO_PIN_OP_TYPES
	uint8_t GPIO_PinAltFunMode;			//
}GPIO_PinConfig_t;


typedef struct{
	GPIO_RegDef_t *pGPIOx; // This pointer holds the base address of the GPIO port.
	GPIO_PinConfig_t GPIO_PinConfig; // This holds GPIO pin configurations
}GPIOx_Handle_t;


//Peripheral Clock Control
void GPIO_PeripheralClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);


/*
 *GPIO port Init and DeInit
 */
void GPIO_Inint(GPIOx_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*GPIO Read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);


/*
 * Interrupt configure and Handle
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandle(uint8_t PinNumber);


#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
