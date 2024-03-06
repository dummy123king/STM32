/*
 *  002LED_with_Button.c
 *
 *  Created on: Mar 1, 2024
 *  Author: Shaik
 */

#include <string.h>
#include "stm32f401xx_gpio_driver.h"

#define HIGH			1
#define LOW				0
#define BUTTON_PRESSED	LOW
uint16_t flag = 0;


void buttonDebouncedelay(void)
{
	for ( int i = 0;  i < 900000; ++i);
}

void EXTI15_10_IRQHandler(void)
{
	buttonDebouncedelay();
	GPIO_IRQHandle(GPIO_PIN_NO_13);
//	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
	if(flag == 0)
	{
		GPIO_WriteToOutputPort(GPIOA, 0xE0);
		flag = 1;
	}
	else
	{
		GPIO_WriteToOutputPort(GPIOA, 0x00);
		flag = 0;
	}
}

//Entry
int main(void)
{
	GPIOx_Handle_t gpioRedLed;
	GPIOx_Handle_t gpioGreenLed;
	GPIOx_Handle_t gpioBlueLed;
	GPIOx_Handle_t gpioButton;

	memset(GPIOA, 0, sizeof(gpioRedLed));
	memset(GPIOA, 0, sizeof(gpioGreenLed));
	memset(GPIOA, 0, sizeof(gpioBlueLed));
	memset(GPIOC, 0, sizeof(gpioButton));

	//LED Init
	gpioGreenLed.pGPIOx = GPIOA;
	gpioGreenLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioGreenLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioGreenLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	gpioGreenLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioGreenLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	GPIO_PeripheralClkCtrl(GPIOA, ENABLE);

	gpioBlueLed.pGPIOx = GPIOA;
	gpioBlueLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	gpioBlueLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioBlueLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	gpioBlueLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioBlueLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	gpioRedLed.pGPIOx = GPIOA;
	gpioRedLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	gpioRedLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioRedLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	gpioRedLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioRedLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;


	GPIO_Inint(&gpioBlueLed);
	GPIO_Inint(&gpioGreenLed);
	GPIO_Inint(&gpioRedLed);

	//Button Init
	gpioButton.pGPIOx = GPIOC;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PD;
	GPIO_PeripheralClkCtrl(GPIOC, ENABLE);
	GPIO_Inint(&gpioButton);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI015_10, NVIC_IRQ_PRIO_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI015_10, ENABLE);

	/* Loop forever */
	for(;;);
}
