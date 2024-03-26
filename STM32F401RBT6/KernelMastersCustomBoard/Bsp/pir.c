/*
 * pir.c
 *
 *  Created on: Mar 26, 2024
 *      Author: shaik
 */

#include "pir.h"

static void mDelay(uint32_t delay)
{
	delay = delay * 1000;
	for (int i = 0; i < delay; ++i) {}
}

void PIR_init(void)
{
	GPIOC_PERI_CLK_EN();
	GPIOx_Handle_t gpioPIRConfig;

	gpioPIRConfig.pGPIOx = PIR_GPIO_PORT;

	gpioPIRConfig.GPIO_PinConfig.GPIO_PinNumber = PIR_GPIO_INPUT_PIN;
	gpioPIRConfig.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioPIRConfig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	gpioPIRConfig.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioPIRConfig.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	GPIO_Init(&gpioPIRConfig);


	for(int i = 0; i < 30; i++)
	{
		mDelay(1000);
	}

}
