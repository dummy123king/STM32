/*
 * pir.h
 *
 *  Created on: Mar 26, 2024
 *      Author: shaik
 */

#ifndef PIR_H_
#define PIR_H_
#include "stm32f401xx_gpio_driver.h"

#define PIR_MOTION_DETECTED		1
#define PIR_IDLE_STATE			0

#define PIR_GPIO_PORT			GPIOC
#define PIR_GPIO_INPUT_PIN 		GPIO_PIN_NO_5


void PIR_init(void);



#endif /* PIR_H_ */
