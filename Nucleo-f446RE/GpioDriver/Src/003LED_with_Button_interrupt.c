/*
 *  002LED_with_Button.c
 *
 *  Created on: Mar 1, 2024
 *  Author: Shaik
 */

#include <string.h>
#include "stm32f446xx_gpio_driver.h"

#define HIGH			1
#define LOW				0
#define BUTTON_PRESSED	LOW

void buttonDebouncedelay(void)
{
	for ( int i = 0;  i < 900000; ++i);
}

void EXTI15_10_IRQHandler(void)
{
	buttonDebouncedelay();
	GPIO_IRQHandle(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}

//Entry
int main(void)
{
	GPIOx_Handle_t gpioLed;
	GPIOx_Handle_t gpioButton;

	memset(GPIOA, 0, sizeof(gpioLed));
	memset(GPIOC, 0, sizeof(gpioButton));

	//LED Init
	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	GPIO_PeripheralClkCtrl(GPIOA, ENABLE);
	GPIO_Inint(&gpioLed);

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
