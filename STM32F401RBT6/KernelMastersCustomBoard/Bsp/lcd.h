/*
 * lcd.h
 *
 *  Created on: Mar 7, 2024
 *      Author: Shaik
 */

#ifndef __LCD_H__
#define __LCD_H__

#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

#define LCD_GPIO_PORT		GPIOB
#define LCD_GPIO_D4			GPIO_PIN_NO_0
#define LCD_GPIO_D5			GPIO_PIN_NO_1
#define LCD_GPIO_D6			GPIO_PIN_NO_2
#define LCD_GPIO_D7			GPIO_PIN_NO_3
#define LCD_GPIO_RS			GPIO_PIN_NO_4
#define LCD_GPIO_RW			GPIO_PIN_NO_5
#define LCD_GPIO_EN			GPIO_PIN_NO_8




/* LCD Commands */
#define LCD_CMD_4DL_2N_5x8F				0x28
#define LCD_CMD_DISP_ON_CURON			0x0E
#define LCD_CMD_INCADD					0x06
#define LCD_CMD_DISP_CLR				0x01
#define LCD_CMD_DISP_RETURN_HOME		0x02


void lcdInit(void);
void lcdSendCmd(uint8_t data);
void lcdSendChar(uint8_t data);
void lcdDispClear(void);
void lcdDispReturnHome(void);
void lcdSetCursor(uint8_t row, int column);
void lcdPrintStr(char *data);

#endif /* __LCD_H__ */
