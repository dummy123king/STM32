/*
 * stm32f446xx.h
 *
 *  Created on: Feb 16, 2024
 *  Author: Imran Shaik
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

typedef uint32_t u32;
#define __vo	volatile

#define BIT_POS_0	0
#define BIT_POS_1	1
#define BIT_POS_2	2
#define BIT_POS_3	3
#define BIT_POS_4	4
#define BIT_POS_5	5
#define BIT_POS_6	6
#define BIT_POS_7	7
#define BIT_POS_8	8
#define BIT_POS_9	9
#define BIT_POS_10	10
#define BIT_POS_11	11
#define BIT_POS_12	12
#define BIT_POS_13	13
#define BIT_POS_14	14
#define BIT_POS_15	15
#define BIT_POS_16	16
#define BIT_POS_17	17
#define BIT_POS_18	18
#define BIT_POS_19	19
#define BIT_POS_20	20
#define BIT_POS_21	21
#define BIT_POS_22	22
#define BIT_POS_23	23
#define BIT_POS_24	24
#define BIT_POS_25	25
#define BIT_POS_26	26
#define BIT_POS_27	27
#define BIT_POS_28	28
#define BIT_POS_29	29
#define BIT_POS_30	30
#define BIT_POS_31	31

#define ENABLE							    1
#define DISABLE								0
#define SET									ENABLE
#define RESET								DISABLE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET


/********************* Base address of FLASH and SRAM Memories*********/
#define FLASH_BASE_ADDR						0x08000000U
#define SRAM_1_BASE_ADDR					0x20000000U //112KB
#define SRAM_2_BASE_ADDR					0x2001C000U //16KB
#define ROM_BASE_ADDR						0x1FFF0000U
#define SRAM								SRAM_1_BASE_ADDR
/**********************************************************************/


/****************Base address of PeriPherals************************/
#define PERIPH_BASE_ADDR					0x40000000U
#define APB1_PERIPH_BASE_ADDR				PERIPH_BASE_ADDR
#define APB2_PERIPH_BASE_ADDR				0x40010000U
#define AHB1_PERIPH_BASE_ADDR				0x40020000U
#define AHB2_PERIPH_BASE_ADDR				0x50000000U
#define AHB3_PERIPH_BASE_ADDR				0x60000000U

///For GPIOx(APBH1) Base addresses 									//Offset
#define GPIOA_BASE_ADDR 					(AHB1_PERIPH_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR						(AHB1_PERIPH_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR						(AHB1_PERIPH_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR						(AHB1_PERIPH_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR						(AHB1_PERIPH_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR						(AHB1_PERIPH_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR						(AHB1_PERIPH_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR						(AHB1_PERIPH_BASE_ADDR + 0x1C00)

#define RCC_BASE_ADDR                       (AHB1_PERIPH_BASE_ADDR + 0x3800)

/***************************APB1 Peripherals ******************************************/
//I2C
#define I2C1_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x5C00)

//SPI
#define SPI2_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x3C00)

//UART
#define UART4_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x4000)
#define UART5_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x5000)

//USART
#define USART2_BASE_ADDR					(APB1_PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR					(APB1_PERIPH_BASE_ADDR + 0x4800)

//Timers
#define TIM2_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x0000)
#define TIM3_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x0400)
#define TIM4_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x0800)
#define TIM5_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x0C00)
#define TIM6_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x1000)
#define TIM7_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x1400)
#define TIM12_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x1800)
#define TIM13_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x1C00)
#define TIM14_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x2000)

//WatchDog Timer
#define WWDG_BASE_ADDR						(APB1_PERIPH_BASE_ADDR + 0x2C00)
/*--------------------------------------------------------------------------------*/



/********************************   APB2 Peripherals    ********************************* **/
//SPI
#define SPI1_BASE_ADDR						(APB2_PERIPH_BASE_ADDR + 0x3000)
#define SPI4_BASE_ADDR						(APB2_PERIPH_BASE_ADDR + 0x3400)

//USART
#define USART1_BASE_ADDR					(APB2_PERIPH_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR					(APB2_PERIPH_BASE_ADDR + 0x1400)

//TIMERs
#define	TIM1_BASE_ADDR						(APB2_PERIPH_BASE_ADDR + 0x0000)
#define TIM8_BASE_ADDR						(APB2_PERIPH_BASE_ADDR + 0x0400)
#define TIM9_BASE_ADDR						(APB2_PERIPH_BASE_ADDR + 0x4000)
#define TIM10_BASE_ADDR						(APB2_PERIPH_BASE_ADDR + 0x4400)
#define TIM11_BASE_ADDR						(APB2_PERIPH_BASE_ADDR + 0x4800)

#define SYSCFG_BASE_ADDR					(APB2_PERIPH_BASE_ADDR + 0x3800)

#define EXTI_BASE_ADDR						(APB2_PERIPH_BASE_ADDR + 0x3C00)

typedef struct
{
	__vo u32 MODER;         /*GPIO port mode register*/                 //Offset Address 0x00
    __vo u32 OTYPER;        /*GPIO port output type register*/          //Offset Address 0x04
    __vo u32 OSPEEDER;      /*GPIO port output speed register*/         //Offset Address 0x08
    __vo u32 PUPDR;         /*GPIO port pull-up/pull-down register*/    //Offset Address 0x0C
    __vo u32 IDR;           /*GPIO port input data register*/           //Offset Address 0x10
    __vo u32 ODR;           /*GPIO port output data register*/          //Offset Address 0x14
    __vo u32 BSRR;          /*GPIO port bit set/reset register*/        //Offset Address 0x18
    __vo u32 LCKR;          /*GPIO port configuration lock register*/   //Offset Address 0x1C
    __vo u32 AFR[2];        /*GPIO alternate function low register*/    //Offset Address AFRL 0x20, AFRH 0x24
}GPIO_RegDef_t;


//Clock Register RCC
typedef struct
{
	__vo u32 CR;                /*clock control register*/                                       //Offset Address 0x00
	__vo u32 PLL_CFGR;    /*RCC PLL configuration register*/                                     //Offset Address 0x04
	__vo u32 CFGR;        /*RCC clock configuration register*/                                   //Offset Address 0x08
	__vo u32 CIR;         /*RCC clock interrupt register*/                                       //Offset Address 0x0C
	__vo u32 AHB1RSTR;    /*RCC AHB1 peripheral reset register*/                                 //Offset Address 0x10
	__vo u32 AHB2RSTR;    /*RCC AHB2 peripheral reset register*/                                 //Offset Address 0x14
	__vo u32 AHB3RSTR;    /*RCC AHB3 peripheral reset register*/                                 //Offset Address 0x18
	__vo u32 RESERVED0;   /*RESERVED*/                                                           //Offset Address 0x1C
	__vo u32 APB1RSTR;    /*RCC APB1 peripheral reset register*/                                 //Offset Address 0x20
	__vo u32 APB2RSTR;    /*RCC APB2 peripheral reset register*/                                 //Offset Address 0x24
	__vo u32 RESERVED1;   /*RESERVED*/                                                           //Offset Address 0x28
	__vo u32 RESERVED2;   /*RESERVED*/                                                           //Offset Address 0x2C
	__vo u32 AHB1ENR;     /*RCC AHB1 peripheral clock enable register*/                          //Offset Address 0x30
	__vo u32 AHB2ENR;     /*RCC AHB2 peripheral clock enable register */                         //Offset Address 0x34
	__vo u32 AHB3ENR;     /*RCC AHB3 peripheral clock enable register*/                          //Offset Address 0x38
	__vo u32 RESERVED3;   /*RESERVED*/                                                           //Offset Address 0x3C
	__vo u32 APB1ENR;     /*RCC APB1 peripheral clock enable register*/                          //Offset Address 0x40
	__vo u32 APB2ENR;     /*RCC APB2 peripheral clock enable register*/                          //Offset Address 0x44
	__vo u32 RESERVED4;   /*RESERVED*/                                                           //Offset Address 0x48
	__vo u32 RESERVED5;   /*RESERVED*/                                                           //Offset Address 0x4C
	__vo u32 AHB1LPENR;   /*RCC AHB1 peripheral clock enable in low power mode register*/        //Offset Address 0x50
	__vo u32 AHB2LPENR;   /*RCC AHB2 peripheral clock enable in low power mode register*/        //Offset Address 0x54
	__vo u32 AHB3LPENR;   /*RCC AHB3 peripheral clock enable in low power mode register*/        //Offset Address 0x58
	__vo u32 RESERVED6;   /*RESERVED*/                                                           //Offset Address 0x5C
	__vo u32 APB1LPENR;   /*RCC APB1 peripheral clock enable in low power mode register*/        //Offset Address 0x60
	__vo u32 APB2LPENR;   /*RCC APB2 peripheral clock enabled in low power mode register*/       //Offset Address 0x64
	__vo u32 RESERVED7;   /*RESERVED*/                                                           //Offset Address 0x68
	__vo u32 RESERVED8;   /*RESERVED*/                                                           //Offset Address 0x6C
	__vo u32 BDCR;        /*RCC Backup domain control register*/                                 //Offset Address 0x70
	__vo u32 CSR;         /*RCC clock control & status register*/                                //Offset Address 0x74
	__vo u32 RESERVED9;   /*RESERVED*/                                                           //Offset Address 0x78
	__vo u32 RESERVED10;  /*RESERVED*/                                                           //Offset Address 0x7C
	__vo u32 SSCGR;       /*RCC spread spectrum clock generation register*/                      //Offset Address 0x80
	__vo u32 PLLI2SCFGR;  /*RCC PLLI2S configuration register*/                                  //Offset Address 0x84
	__vo u32 PLLSAICFGR;  /*RCC PLL configuration register*/                                     //Offset Address 0x88
	__vo u32 DCKCFGR;     /*RCC dedicated clock configuration register*/                         //Offset Address 0x8C
	__vo u32 CKGATENR;    /*RCC clocks gated enable register*/                                   //Offset Address 0x90
	__vo u32 DCKCFGR2;    /*RCC dedicated clocks configuration register 2*/                      //Offset Address 0x94
}RCC_RegDef_t;


//EXTI register structure
typedef struct
{
	__vo u32 IMR;          /* Interrupt mask register */                 //Offset Address 0x00
    __vo u32 EMR;          /* Event mask register */			         //Offset Address 0x04
    __vo u32 RTSR;         /* Rising trigger selection register */       //Offset Address 0x08
    __vo u32 FTSR;         /* Falling trigger selection register */      //Offset Address 0x0C
    __vo u32 SWIER;        /* Software interrupt event register */       //Offset Address 0x10
    __vo u32 PR;           /* Pending register */         			     //Offset Address 0x14
}EXTI_RegDef_t;


//SYSCFG register structure
typedef struct
{
	__vo u32 MEMRMP;       /*SYSCFG memory re-map register */                 				//Offset Address 0x00
    __vo u32 PMC;          /*SYSCFG peripheral mode configuration register*/			    //Offset Address 0x04
    __vo u32 EXTICR[4];    /*SYSCFG external interrupt configuration registers 1 to 4*/   	//Offset Address 0x08 to 0x14
    u32 resrved1[2];	   /* Reserved from 0x14 to 0x18*/
    __vo u32 CMPCR;        /*Compensation cell control register*/       					//Offset Address 0x20
    u32 resrved2[2];	   /* Reserved from 0x24 to 0x28 */
    __vo u32 CFGR;         /*SYSCFG configuration register */         			   			//Offset Address 0x2C
}SYSCFG_RegDef_t;



/*
*Peripheral Definitions (Peripheral base address typecasted to xxx_RegDef_t)
*/

#define GPIOA                                  ((GPIO_RegDef_t *)(GPIOA_BASE_ADDR))
#define GPIOB                                  ((GPIO_RegDef_t *)(GPIOB_BASE_ADDR))
#define GPIOC                                  ((GPIO_RegDef_t *)(GPIOC_BASE_ADDR))
#define GPIOD                                  ((GPIO_RegDef_t *)(GPIOD_BASE_ADDR))
#define GPIOE                                  ((GPIO_RegDef_t *)(GPIOE_BASE_ADDR))
#define GPIOF                                  ((GPIO_RegDef_t *)(GPIOF_BASE_ADDR))
#define GPIOG                                  ((GPIO_RegDef_t *)(GPIOG_BASE_ADDR))
#define GPIOH                                  ((GPIO_RegDef_t *)(GPIOH_BASE_ADDR))

//Clock Control
#define RCC                                    ((RCC_RegDef_t *)(RCC_BASE_ADDR))


//EXTI control(External interrupt control)
#define EXTI								   ((EXTI_RegDef_t *)(EXTI_BASE_ADDR))


//SYSCFG control Register
#define SYSCFG								   ((SYSCFG_RegDef_t *)(SYSCFG_BASE_ADDR))

//Enable Clock for GPIOx Peripherals
#define GPIOA_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << 7))


//Enable clock for I2Cx peripherals
#define I2C1_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 21))
#define I2C2_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 22))
#define I2C3_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 23))


//Enable Clock for SPIx peripherals
#define SPI1_PERI_CLK_EN()                     (RCC->APB2ENR |= (1 << 12))
#define SPI2_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 14))
#define SPI3_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 15))
#define SPI4_PERI_CLK_EN()                     (RCC->APB2ENR |= (1 << 13))


//Enable Clock for USARTx Peripherals
#define USART1_PERI_CLK_EN()                   (RCC->APB2ENR |= (1 << 1 ))
#define USART2_PERI_CLK_EN()                   (RCC->APB1ENR |= (1 << 17))
#define USART3_PERI_CLK_EN()                   (RCC->APB1ENR |= (1 << 18))
#define USART6_PERI_CLK_EN()                   (RCC->APB2ENR |= (1 << 5 ))


//Enable Clock for UARTx Peripherals
#define UART4_PERI_CLK_EN()                    (RCC->APB1ENR |= (1 << 19))
#define UART5_PERI_CLK_EN()                    (RCC->APB1ENR |= (1 << 20))


//Enable Clock for TIMx peripherals
#define TIM1_PERI_CLK_EN()                     (RCC->APB2ENR |= (1 << 0))
#define TIM2_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 0))
#define TIM3_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 1))
#define TIM4_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 2))
#define TIM5_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 3))
#define TIM6_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 4))
#define TIM7_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << 5))
#define TIM8_PERI_CLK_EN()                     (RCC->APB2ENR |= (1 << 1))
#define TIM9_PERI_CLK_EN()                     (RCC->APB2ENR |= (1 << 16))
#define TIM10_PERI_CLK_EN()                    (RCC->APB2ENR |= (1 << 17))
#define TIM11_PERI_CLK_EN()                    (RCC->APB2ENR |= (1 << 18))
#define TIM12_PERI_CLK_EN()                    (RCC->APB1ENR |= (1 << 6))
#define TIM13_PERI_CLK_EN()                    (RCC->APB1ENR |= (1 << 7))
#define TIM14_PERI_CLK_EN()                    (RCC->APB1ENR |= (1 << 8))


//Enable Clock for SYSCFG peripheral
#define SYSCFG_PERI_CLK_EN()                   (RCC->APB2ENR |= (1 << 14))




//Macros to reset GPIOx peripherals
#define GPIOA_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)


//Disable Clock for GPIOx Peripherals
#define GPIOA_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << 7))


//Disable clock for I2Cx peripherals
#define I2C1_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 23))


//Disable Clock for SPIx peripherals
#define SPI1_PERI_CLK_DI()                     (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PERI_CLK_DI()                     (RCC->APB2ENR &= ~(1 << 13))


//Disable Clock for USARTx Peripherals
#define USART1_PERI_CLK_DI()                   (RCC->APB2ENR &= ~(1 << 1 ))
#define USART2_PERI_CLK_DI()                   (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PERI_CLK_DI()                   (RCC->APB1ENR &= ~(1 << 18))
#define USART6_PERI_CLK_DI()                   (RCC->APB2ENR &= ~(1 << 5 ))


//Disable Clock for UARTx Peripherals
#define UART4_PERI_CLK_DI()                    (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PERI_CLK_DI()                    (RCC->APB1ENR &= ~(1 << 20))


//Disable Clock for TIMx peripherals
#define TIM1_PERI_CLK_DI()                     (RCC->APB2ENR &= ~(1 << 0))
#define TIM2_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 0))
#define TIM3_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 1))
#define TIM4_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 2))
#define TIM5_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 3))
#define TIM6_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 4))
#define TIM7_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << 5))
#define TIM8_PERI_CLK_DI()                     (RCC->APB2ENR &= ~(1 << 1))
#define TIM9_PERI_CLK_DI()                     (RCC->APB2ENR &= ~(1 << 16))
#define TIM10_PERI_CLK_DI()                    (RCC->APB2ENR &= ~(1 << 17))
#define TIM11_PERI_CLK_DI()                    (RCC->APB2ENR &= ~(1 << 18))
#define TIM12_PERI_CLK_DI()                    (RCC->APB1ENR &= ~(1 << 6))
#define TIM13_PERI_CLK_DI()                    (RCC->APB1ENR &= ~(1 << 7))
#define TIM14_PERI_CLK_DI()                    (RCC->APB1ENR &= ~(1 << 8))


//Disable Clock for SYSCFG peripheral
#define SYSCFG_PERI_CLK_DI()                   (RCC->APB2ENR &= ~(1 << 14))


#define GPIO_BASE_ADDR_TO_CODE(x)				( (x == GPIOA)?0:\
												 (x == GPIOA)?1:\
												 (x == GPIOA)?2:\
												 (x == GPIOA)?3:\
												 (x == GPIOA)?4:\
										  		 (x == GPIOA)?5:\
												 (x == GPIOA)?6:\
		  										 (x == GPIOA)?7:0)

#endif /* INC_STM32F446XX_H_ */
