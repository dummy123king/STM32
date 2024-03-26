#include "stm32f401xx_gpio_driver.h"
#include "lcd.h"
#include "pir.h"

#define ENABLE_BUZZER DISABLE

static void mDelay(uint32_t delay)
{
	delay = delay * 1000;
	for (int i = 0; i < delay; ++i) {}
}


// Entry
int main(void)
{
	lcdInit();

#if ENABLE_BUZZER == ENABLE

	GPIOx_Handle_t gpioBuzzer;

    gpioBuzzer.pGPIOx = GPIOB;

    gpioBuzzer.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    gpioBuzzer.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioBuzzer.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    gpioBuzzer.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioBuzzer.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

    GPIO_Init(&gpioBuzzer);

#endif

	lcdDispClear();
	lcdPrintStr("PIR Calib...");

	PIR_init();
	lcdDispClear();
	lcdPrintStr("PIR ACTIVE");

    /* Loop forever */
    for (;;)
    {
    	if(GPIO_ReadFromInputPin(PIR_GPIO_PORT, PIR_GPIO_INPUT_PIN) == PIR_MOTION_DETECTED)
    	{
    		lcdDispClear();
    		lcdPrintStr("Motion Detected");
#if ENABLE_BUZZER == ENABLE
    		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, GPIO_PIN_SET);
#endif
            mDelay(1000);

    	}
    	else if(GPIO_ReadFromInputPin(PIR_GPIO_PORT, PIR_GPIO_INPUT_PIN) == PIR_IDLE_STATE)
    	{
    		lcdDispClear();
    		lcdPrintStr("IDLE");
#if ENABLE_BUZZER == ENABLE
    		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, GPIO_PIN_RESET);
#endif
    		mDelay(1000);
    	}
    }
}
