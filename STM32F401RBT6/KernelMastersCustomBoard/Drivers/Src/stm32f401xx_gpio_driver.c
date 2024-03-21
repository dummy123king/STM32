/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Feb 20, 2024
 *  Author: Shaik
 */

#include "stm32f401xx_gpio_driver.h"


void GPIO_PeripheralClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERI_CLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PERI_CLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PERI_CLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PERI_CLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PERI_CLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PERI_CLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PERI_CLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PERI_CLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERI_CLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PERI_CLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PERI_CLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PERI_CLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PERI_CLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PERI_CLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PERI_CLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PERI_CLK_DI();
		}
	}
}


void GPIO_Init(GPIOx_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//Configure GPIO MODE
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Non Interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) * 2);
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) * 2);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		//Configure FALING TRIGGER SLECTION REGISTER (FTSR)
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//Set FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//Clear RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//Configure RISING TRIGGER SLECTION REGISTER (RTSR)
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//Set RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//Clear FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//Configure FALING and RISING SLECTION REGISTER (FTSR and RTSR)
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT)
		{
			//Set FTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Set FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PERI_CLK_EN();
		SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4);

		//Enable the EXTI interrupt delivery using IMR(Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	//Configure SPEED
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) * 2);
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	//Configure Pull up and pull down
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl << ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) * 2);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//Configure output OPtype
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	//Configure pin alternate function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_REST();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_REST();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_REST();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_REST();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_REST();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_REST();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_REST();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_REST();
	}
}


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	return ((uint8_t)((pGPIOx->IDR >> pinNumber) & 0x1 ));
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return ((uint16_t)(pGPIOx->IDR));
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (0x1 << pinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(0x1 << pinNumber);
	}
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}


void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if (IRQNumber <= 31) //program ISER0
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 32 && IRQNumber <= 63) //program ISER1
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber <= 95) //program ISER2
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if (IRQNumber <= 31) //program ICER0
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 32 && IRQNumber <= 63) //program ICER1
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber <= 95) //program ICER2
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//Locate IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}


void GPIO_IRQHandle(uint8_t PinNumber)
{
	//Clear the EXTI PR register Corresponding to the PIN

	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
