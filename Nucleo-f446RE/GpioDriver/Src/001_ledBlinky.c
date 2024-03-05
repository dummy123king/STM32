
#include "stm32f446xx_gpio_driver.h"

void delay(void)
{
	for ( int i = 0;  i < 1000000; ++i);
}

//Entry
int main(void)
{
	GPIOx_Handle_t gpioLed;

	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	GPIO_PeripheralClkCtrl(GPIOA, ENABLE);
	GPIO_Inint(&gpioLed);

	/* Loop forever */
	for(;;)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();

//		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);
//		delay();
//		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_RESET);
//		delay();

//		GPIO_WriteToOutputPort(GPIOA, 0x20);
//		delay();
//		GPIO_WriteToOutputPort(GPIOA, 0x00);
//		delay();
	}
}
