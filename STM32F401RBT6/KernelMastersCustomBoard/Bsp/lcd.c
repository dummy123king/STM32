/*
 * lcd.c
 *
 *  Created on: Mar 7, 2024
 *      Author: Shaik
 */


#include "lcd.h"
#include <string.h>

void mDelay(uint32_t delay)
{
	delay = delay * 1000;
	for (int i = 0; i < delay; ++i) {

	}
}

void uDelay(uint32_t delay)
{
	delay = delay * 1600;
	for (int i = 0; i < delay; ++i) {

	}
}

static void enableLCD(void)
{
	//Set the EN pin
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	uDelay(1);

	//Reset the EN PIN
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	uDelay(1);
}

//Write 4-bit(Nibble) data to LCD (D4, D5, D6, D7)
static void writeDataToLCD(uint8_t data)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, (data >> 0) & 1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, (data >> 1) & 1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, (data >> 2) & 1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, (data >> 3) & 1);

	enableLCD();
}


void lcdSendCmd(uint8_t data)
{
	/*RS = 0, for LCD Command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	writeDataToLCD(data >> 4);
	writeDataToLCD(data & 0x0F);

}


void lcdSendChar(uint8_t data)
{
	/*RS = 0, for LCD Command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	writeDataToLCD(data >> 4);
	writeDataToLCD(data & 0x0F);
}


void lcdDispClear(void)
{
	//Display Clear Command
	lcdSendCmd(LCD_CMD_DISP_CLR);
	mDelay(10);
}


void lcdDispCursorON(void)
{
	//Display Clear Command
	lcdSendCmd(LCD_CMD_DISP_ON_CURON);;
	mDelay(10);
}


void lcdDispReturnHome(void)
{
	lcdSendCmd(LCD_CMD_DISP_RETURN_HOME);
	mDelay(5);
}


void lcdSetCursor(uint8_t row, int column)
{
	column--;
	switch (row) {
		case 1:
			column |= 0x80;
			lcdSendCmd(column);
			break;

		case 2:
			column |= 0xC0;
			lcdSendCmd(column);
			break;
		default:
			break;
	}
}


void lcdPrintStr(char *data)
{
	while(*data != '\0')
	{
		lcdSendChar((uint8_t)*data);
		data++;
	}
}


void lcdInit(void)
{
	GPIOB_PERI_CLK_EN();
	GPIOx_Handle_t gpioLCDConfig;

	gpioLCDConfig.pGPIOx = LCD_GPIO_PORT;

	gpioLCDConfig.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	gpioLCDConfig.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLCDConfig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	gpioLCDConfig.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLCDConfig.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	GPIO_Init(&gpioLCDConfig);

	gpioLCDConfig.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&gpioLCDConfig);

	gpioLCDConfig.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Init(&gpioLCDConfig);

	gpioLCDConfig.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&gpioLCDConfig);

	gpioLCDConfig.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&gpioLCDConfig);

	gpioLCDConfig.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&gpioLCDConfig);

	gpioLCDConfig.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&gpioLCDConfig);


	//Reset all the PINS
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);
	uDelay(100);

	lcdSendCmd(0x33);
	uDelay(100);

	lcdSendCmd(0x32);
	uDelay(100);

	lcdSendCmd(0x0C);
	lcdDispClear();
}

