#include "stm32f401xx_gpio_driver.h"

#define ENABLE_BUZZER ENABLE

#define HIGH 1
#define LOW 0
#define BUTTON_PRESSED LOW

void buttonDebouncedelay(void)
{
    for (int i = 0; i < 300000; ++i);
}

void delay(void)
{
    for (int i = 0; i < 1000000; ++i);
}

//Interrupt Handler
void EXTI9_5_IRQHandler(void)
{
	buttonDebouncedelay();
	GPIO_IRQHandle(GPIO_PIN_NO_9);
    GPIO_WriteToOutputPort(GPIOB, 0x6000);
}

// Entry
int main(void)
{
    GPIOx_Handle_t gpioGreenLed;
    GPIOx_Handle_t gpioRedLed;
    GPIOx_Handle_t gpioButtonUp;
    GPIOx_Handle_t gpioButtonDn;

    GPIO_PeripheralClkCtrl(GPIOB, ENABLE);
    GPIO_PeripheralClkCtrl(GPIOC, ENABLE);

#if ENABLE_BUZZER == ENABLE
    GPIOx_Handle_t gpioBuzzer;
#endif

    gpioGreenLed.pGPIOx = GPIOB;
    gpioGreenLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    gpioGreenLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioGreenLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    gpioGreenLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioGreenLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

    gpioRedLed.pGPIOx = GPIOB;
    gpioRedLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    gpioRedLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioRedLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    gpioRedLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioRedLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

#if ENABLE_BUZZER == ENABLE
    gpioBuzzer.pGPIOx = GPIOB;
    gpioBuzzer.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    gpioBuzzer.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioBuzzer.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    gpioBuzzer.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioBuzzer.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
    GPIO_Inint(&gpioBuzzer);
#endif

    gpioButtonUp.pGPIOx = GPIOC;
    gpioButtonUp.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    gpioButtonUp.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    gpioButtonUp.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    gpioButtonUp.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;

    gpioButtonDn.pGPIOx = GPIOC;
    gpioButtonDn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    gpioButtonDn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FRT;
    gpioButtonDn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    gpioButtonDn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;

    GPIO_Inint(&gpioGreenLed);
    GPIO_Inint(&gpioRedLed);
    GPIO_WriteToOutputPort(GPIOB, 0x6000);
    GPIO_Inint(&gpioButtonUp);
    GPIO_Inint(&gpioButtonDn);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO_14);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

    /* Loop forever */
    for (;;)
    {
        if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_8) == BUTTON_PRESSED)
        {
            buttonDebouncedelay();
            GPIO_WriteToOutputPort(GPIOB, 0x0000);
#if ENABLE_BUZZER == ENABLE
            GPIO_WriteToOutputPort(GPIOB, 0x1000);
#endif
        }
    }
}
