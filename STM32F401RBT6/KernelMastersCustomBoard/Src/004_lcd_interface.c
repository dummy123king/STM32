#include "stm32f401xx_gpio_driver.h"
#include "lcd.h"
#include "stdio.h"

void delay(uint32_t delay)
{
	delay = delay * 5000000;
    for (int i = 0; i < delay ; ++i)
    {}
}

int __io_putchar(int ch)
{
	lcdSendChar(ch);
	return ch;
}


// Entry
int main(void)
{
	lcdInit();
	lcdDispClear();

	lcdSetCursor(1, 6);
	lcdPrintStr("Hello");

	lcdSetCursor(2, 3);
	lcdPrintStr("Mirafra Tech");

    /* Loop forever */
    for (;;)
    {


    }
}
