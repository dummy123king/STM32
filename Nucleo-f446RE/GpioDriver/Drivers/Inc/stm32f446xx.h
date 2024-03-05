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


/**
 * ARM cortex Mx processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0						( (__vo u32*) 0xE000E100 )
#define NVIC_ISER1						( (__vo u32*) 0xE000E104 )
#define NVIC_ISER2						( (__vo u32*) 0xE000E108 )
#define NVIC_ISER3						( (__vo u32*) 0xE000E10C )
#define NVIC_ISER4						( (__vo u32*) 0xE000E110 )
#define NVIC_ISER5						( (__vo u32*) 0xE000E114 )
#define NVIC_ISER6						( (__vo u32*) 0xE000E118 )
#define NVIC_ISER7						( (__vo u32*) 0xE000E11C )


/**
 * ARM cortex Mx processor NVIC ICERx Register Addresses
 */
#define NVIC_ICER0						( (__vo u32*) 0xE000E180 )
#define NVIC_ICER1						( (__vo u32*) 0xE000E184 )
#define NVIC_ICER2						( (__vo u32*) 0xE000E188 )
#define NVIC_ICER3						( (__vo u32*) 0xE000E18C )
#define NVIC_ICER4						( (__vo u32*) 0xE000E190 )
#define NVIC_ICER5						( (__vo u32*) 0xE000E194 )
#define NVIC_ICER6						( (__vo u32*) 0xE000E198 )
#define NVIC_ICER7						( (__vo u32*) 0xE000E19C )


/**
 * ARM cortex Mx processor NVIC Interrupt priority Register Addresses
 */
#define NVIC_PR_BASE_ADDR				( (__vo u32*) 0xE000E400 )
#define NO_PR_BITS_IMPLEMENTED			4


enum PRIORITIES {
	NVIC_IRQ_PRIO_0,
	NVIC_IRQ_PRIO_1,
	NVIC_IRQ_PRIO_2,
	NVIC_IRQ_PRIO_3,
	NVIC_IRQ_PRIO_4,
	NVIC_IRQ_PRIO_5,
	NVIC_IRQ_PRIO_6,
	NVIC_IRQ_PRIO_7,
	NVIC_IRQ_PRIO_8,
	NVIC_IRQ_PRIO_9,
	NVIC_IRQ_PRIO_10,
	NVIC_IRQ_PRIO_11,
	NVIC_IRQ_PRIO_12,
	NVIC_IRQ_PRIO_13,
	NVIC_IRQ_PRIO_14,
	NVIC_IRQ_PRIO_15
};

//Bit positions
enum BIT_POSITIONS{
	BIT_POS_0,
	BIT_POS_1,
	BIT_POS_2,
	BIT_POS_3,
	BIT_POS_4,
	BIT_POS_5,
	BIT_POS_6,
	BIT_POS_7,
	BIT_POS_8,
	BIT_POS_9,
	BIT_POS_10,
	BIT_POS_11,
	BIT_POS_12,
	BIT_POS_13,
	BIT_POS_14,
	BIT_POS_15,
	BIT_POS_16,
	BIT_POS_17,
	BIT_POS_18,
	BIT_POS_19,
	BIT_POS_20,
	BIT_POS_21,
	BIT_POS_22,
	BIT_POS_23,
	BIT_POS_24,
	BIT_POS_25,
	BIT_POS_26,
	BIT_POS_27,
	BIT_POS_28,
	BIT_POS_29,
	BIT_POS_30,
	BIT_POS_31,
};

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
	u32 RESERVED0;		  /*RESERVED*/                                                           //Offset Address 0x1C
	__vo u32 APB1RSTR;    /*RCC APB1 peripheral reset register*/                                 //Offset Address 0x20
	__vo u32 APB2RSTR;    /*RCC APB2 peripheral reset register*/                                 //Offset Address 0x24
	u32 RESERVED1;        /*RESERVED*/                                                           //Offset Address 0x28
	u32 RESERVED2;  	  /*RESERVED*/                                                           //Offset Address 0x2C
	__vo u32 AHB1ENR;     /*RCC AHB1 peripheral clock enable register*/                          //Offset Address 0x30
	__vo u32 AHB2ENR;     /*RCC AHB2 peripheral clock enable register */                         //Offset Address 0x34
	__vo u32 AHB3ENR;     /*RCC AHB3 peripheral clock enable register*/                          //Offset Address 0x38
	u32 RESERVED3;	      /*RESERVED*/                                                           //Offset Address 0x3C
	__vo u32 APB1ENR;     /*RCC APB1 peripheral clock enable register*/                          //Offset Address 0x40
	__vo u32 APB2ENR;     /*RCC APB2 peripheral clock enable register*/                          //Offset Address 0x44
	u32 RESERVED4;   	  /*RESERVED*/                                                           //Offset Address 0x48
	u32 RESERVED5;   	  /*RESERVED*/                                                           //Offset Address 0x4C
	__vo u32 AHB1LPENR;   /*RCC AHB1 peripheral clock enable in low power mode register*/        //Offset Address 0x50
	__vo u32 AHB2LPENR;   /*RCC AHB2 peripheral clock enable in low power mode register*/        //Offset Address 0x54
	__vo u32 AHB3LPENR;   /*RCC AHB3 peripheral clock enable in low power mode register*/        //Offset Address 0x58
	u32 RESERVED6;        /*RESERVED*/                                                           //Offset Address 0x5C
	__vo u32 APB1LPENR;   /*RCC APB1 peripheral clock enable in low power mode register*/        //Offset Address 0x60
	__vo u32 APB2LPENR;   /*RCC APB2 peripheral clock enabled in low power mode register*/       //Offset Address 0x64
	u32 RESERVED7; 		  /*RESERVED*/                                                           //Offset Address 0x68
	u32 RESERVED8;   	  /*RESERVED*/                                                           //Offset Address 0x6C
	__vo u32 BDCR;        /*RCC Backup domain control register*/                                 //Offset Address 0x70
	__vo u32 CSR;         /*RCC clock control & status register*/                                //Offset Address 0x74
	u32 RESERVED9;   	  /*RESERVED*/                                                           //Offset Address 0x78
	u32 RESERVED10;  	  /*RESERVED*/                                                           //Offset Address 0x7C
	__vo u32 SSCGR;       /*RCC spread spectrum clock generation register*/                      //Offset Address 0x80
	__vo u32 PLLI2SCFGR;  /*RCC PLLI2S configuration register*/                                  //Offset Address 0x84
	__vo u32 PLLSAICFGR;  /*RCC PLL configuration register*/                                     //Offset Address 0x88
	__vo u32 DCKCFGR;     /*RCC dedicated clock configuration register*/                         //Offset Address 0x8C
	__vo u32 CKGATENR;    /*RCC clocks gated enable register*/                                   //Offset Address 0x90
	__vo u32 DCKCFGR2;    /*RCC dedicated clocks configuration register 2*/                      //Offset Address 0x94
}RCC_RegDef_t;


//EXTI register structure
typedef struct{
	__vo u32 IMR;          /* Interrupt mask register */                 //Offset Address 0x00
    __vo u32 EMR;          /* Event mask register */			         //Offset Address 0x04
    __vo u32 RTSR;         /* Rising trigger selection register */       //Offset Address 0x08
    __vo u32 FTSR;         /* Falling trigger selection register */      //Offset Address 0x0C
    __vo u32 SWIER;        /* Software interrupt event register */       //Offset Address 0x10
    __vo u32 PR;           /* Pending register */         			     //Offset Address 0x14
}EXTI_RegDef_t;


//SYSCFG register structure
typedef struct{
	__vo u32 MEMRMP;       /*SYSCFG memory re-map register */                 				//Offset Address 0x00
    __vo u32 PMC;          /*SYSCFG peripheral mode configuration register*/			    //Offset Address 0x04
    __vo u32 EXTICR[4];    /*SYSCFG external interrupt configuration registers 1 to 4*/   	//Offset Address 0x08 to 0x14
    u32 resrved1[2];	   /* Reserved from 0x14 to 0x18*/
    __vo u32 CMPCR;        /*Compensation cell control register*/       					//Offset Address 0x20
    u32 resrved2[2];	   /* Reserved from 0x24 to 0x28 */
    __vo u32 CFGR;         /*SYSCFG configuration register */         			   			//Offset Address 0x2C
}SYSCFG_RegDef_t;



/*
*Peripheral Definitions (Peripheral base address type-casted to xxx_RegDef_t)
*/

#define GPIOA                                  ((GPIO_RegDef_t *) (GPIOA_BASE_ADDR))
#define GPIOB                                  ((GPIO_RegDef_t *) (GPIOB_BASE_ADDR))
#define GPIOC                                  ((GPIO_RegDef_t *) (GPIOC_BASE_ADDR))
#define GPIOD                                  ((GPIO_RegDef_t *) (GPIOD_BASE_ADDR))
#define GPIOE                                  ((GPIO_RegDef_t *) (GPIOE_BASE_ADDR))
#define GPIOF                                  ((GPIO_RegDef_t *) (GPIOF_BASE_ADDR))
#define GPIOG                                  ((GPIO_RegDef_t *) (GPIOG_BASE_ADDR))
#define GPIOH                                  ((GPIO_RegDef_t *) (GPIOH_BASE_ADDR))

//Clock Control
#define RCC                                    ((RCC_RegDef_t *) (RCC_BASE_ADDR))


//EXTI control(External interrupt control)
#define EXTI								   ((EXTI_RegDef_t *) (EXTI_BASE_ADDR))


//SYSCFG control Register
#define SYSCFG								   ((SYSCFG_RegDef_t *) (SYSCFG_BASE_ADDR))

//Enable Clock for GPIOx Peripherals
#define GPIOA_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << BIT_POS_0))
#define GPIOB_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << BIT_POS_1))
#define GPIOC_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << BIT_POS_2))
#define GPIOD_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << BIT_POS_3))
#define GPIOE_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << BIT_POS_4))
#define GPIOF_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << BIT_POS_5))
#define GPIOG_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << BIT_POS_6))
#define GPIOH_PERI_CLK_EN()                    (RCC->AHB1ENR |= (1 << BIT_POS_7))


//Enable clock for I2Cx peripherals
#define I2C1_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_21))
#define I2C2_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_22))
#define I2C3_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_23))


//Enable Clock for SPIx peripherals
#define SPI1_PERI_CLK_EN()                     (RCC->APB2ENR |= (1 << BIT_POS_12))
#define SPI2_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_14))
#define SPI3_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_15))
#define SPI4_PERI_CLK_EN()                     (RCC->APB2ENR |= (1 << BIT_POS_13))


//Enable Clock for USARTx Peripherals
#define USART1_PERI_CLK_EN()                   (RCC->APB2ENR |= (1 << BIT_POS_1 ))
#define USART2_PERI_CLK_EN()                   (RCC->APB1ENR |= (1 << BIT_POS_17))
#define USART3_PERI_CLK_EN()                   (RCC->APB1ENR |= (1 << BIT_POS_18))
#define USART6_PERI_CLK_EN()                   (RCC->APB2ENR |= (1 << BIT_POS_5 ))


//Enable Clock for UARTx Peripherals
#define UART4_PERI_CLK_EN()                    (RCC->APB1ENR |= (1 << BIT_POS_19))
#define UART5_PERI_CLK_EN()                    (RCC->APB1ENR |= (1 << BIT_POS_20))


//Enable Clock for TIMx peripherals
#define TIM1_PERI_CLK_EN()                     (RCC->APB2ENR |= (1 << BIT_POS_0))
#define TIM2_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_0))
#define TIM3_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_1))
#define TIM4_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_2))
#define TIM5_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_3))
#define TIM6_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_4))
#define TIM7_PERI_CLK_EN()                     (RCC->APB1ENR |= (1 << BIT_POS_5))
#define TIM8_PERI_CLK_EN()                     (RCC->APB2ENR |= (1 << BIT_POS_1))
#define TIM9_PERI_CLK_EN()                     (RCC->APB2ENR |= (1 << BIT_POS_16))
#define TIM10_PERI_CLK_EN()                    (RCC->APB2ENR |= (1 << BIT_POS_17))
#define TIM11_PERI_CLK_EN()                    (RCC->APB2ENR |= (1 << BIT_POS_18))
#define TIM12_PERI_CLK_EN()                    (RCC->APB1ENR |= (1 << BIT_POS_6))
#define TIM13_PERI_CLK_EN()                    (RCC->APB1ENR |= (1 << BIT_POS_7))
#define TIM14_PERI_CLK_EN()                    (RCC->APB1ENR |= (1 << BIT_POS_8))


//Enable Clock for SYSCFG peripheral
#define SYSCFG_PERI_CLK_EN()                   (RCC->APB2ENR |= (1 << BIT_POS_14))



//Macros to reset GPIOx peripherals
#define GPIOA_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << BIT_POS_0)); (RCC->AHB1RSTR &= ~(1 << BIT_POS_0));}while(0)
#define GPIOB_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << BIT_POS_1)); (RCC->AHB1RSTR &= ~(1 << BIT_POS_1));}while(0)
#define GPIOC_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << BIT_POS_2)); (RCC->AHB1RSTR &= ~(1 << BIT_POS_2));}while(0)
#define GPIOD_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << BIT_POS_3)); (RCC->AHB1RSTR &= ~(1 << BIT_POS_3));}while(0)
#define GPIOE_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << BIT_POS_4)); (RCC->AHB1RSTR &= ~(1 << BIT_POS_4));}while(0)
#define GPIOF_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << BIT_POS_5)); (RCC->AHB1RSTR &= ~(1 << BIT_POS_5));}while(0)
#define GPIOG_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << BIT_POS_6)); (RCC->AHB1RSTR &= ~(1 << BIT_POS_6));}while(0)
#define GPIOH_REG_REST()                    do {(RCC->AHB1RSTR |= (1 << BIT_POS_7)); (RCC->AHB1RSTR &= ~(1 << BIT_POS_7));}while(0)


//Disable Clock for GPIOx Peripherals
#define GPIOA_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << BIT_POS_0))
#define GPIOB_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << BIT_POS_1))
#define GPIOC_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << BIT_POS_2))
#define GPIOD_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << BIT_POS_3))
#define GPIOE_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << BIT_POS_4))
#define GPIOF_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << BIT_POS_5))
#define GPIOG_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << BIT_POS_6))
#define GPIOH_PERI_CLK_DI()                    (RCC->AHB1ENR &= ~(1 << BIT_POS_7))

//Disable clock for I2Cx peripherals
#define I2C1_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_21))
#define I2C2_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_22))
#define I2C3_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_23))


//Disable Clock for SPIx peripherals
#define SPI1_PERI_CLK_DI()                     (RCC->APB2ENR &= ~(1 << BIT_POS_12))
#define SPI2_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_14))
#define SPI3_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_15))
#define SPI4_PERI_CLK_DI()                     (RCC->APB2ENR &= ~(1 << BIT_POS_13))


//Disable Clock for USARTx Peripherals
#define USART1_PERI_CLK_DI()                   (RCC->APB2ENR &= ~(1 << BIT_POS_1 ))
#define USART2_PERI_CLK_DI()                   (RCC->APB1ENR &= ~(1 << BIT_POS_17))
#define USART3_PERI_CLK_DI()                   (RCC->APB1ENR &= ~(1 << BIT_POS_18))
#define USART6_PERI_CLK_DI()                   (RCC->APB2ENR &= ~(1 << BIT_POS_5 ))


//Disable Clock for UARTx Peripherals
#define UART4_PERI_CLK_DI()                    (RCC->APB1ENR &= ~(1 << BIT_POS_19))
#define UART5_PERI_CLK_DI()                    (RCC->APB1ENR &= ~(1 << BIT_POS_20))


//Disable Clock for TIMx peripherals
#define TIM1_PERI_CLK_DI()                     (RCC->APB2ENR &= ~(1 << BIT_POS_0))
#define TIM2_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_0))
#define TIM3_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_1))
#define TIM4_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_2))
#define TIM5_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_3))
#define TIM6_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_4))
#define TIM7_PERI_CLK_DI()                     (RCC->APB1ENR &= ~(1 << BIT_POS_5))
#define TIM8_PERI_CLK_DI()                     (RCC->APB2ENR &= ~(1 << BIT_POS_1))
#define TIM9_PERI_CLK_DI()                     (RCC->APB2ENR &= ~(1 << BIT_POS_16))
#define TIM10_PERI_CLK_DI()                    (RCC->APB2ENR &= ~(1 << BIT_POS_17))
#define TIM11_PERI_CLK_DI()                    (RCC->APB2ENR &= ~(1 << BIT_POS_18))
#define TIM12_PERI_CLK_DI()                    (RCC->APB1ENR &= ~(1 << BIT_POS_6))
#define TIM13_PERI_CLK_DI()                    (RCC->APB1ENR &= ~(1 << BIT_POS_7))
#define TIM14_PERI_CLK_DI()                    (RCC->APB1ENR &= ~(1 << BIT_POS_8))


//Disable Clock for SYSCFG peripheral
#define SYSCFG_PERI_CLK_DI()                   (RCC->APB2ENR &= ~(1 << BIT_POS_14))


#define GPIO_BASE_ADDR_TO_CODE(x)				( (x == GPIOA)?0:\
												 (x == GPIOB)?1:\
												 (x == GPIOC)?2:\
												 (x == GPIOD)?3:\
												 (x == GPIOE)?4:\
										  		 (x == GPIOF)?5:\
												 (x == GPIOG)?6:\
		  										 (x == GPIOH)?7:0)

//External iterrupt IRQ_NO
#define IRQ_NO_EXTI0							6
#define IRQ_NO_EXTI1							7
#define IRQ_NO_EXTI2							8
#define IRQ_NO_EXTI3							9
#define IRQ_NO_EXTI4							10
#define IRQ_NO_EXTI9_5							23
#define IRQ_NO_EXTI015_10						40





#endif /* INC_STM32F446XX_H_ */
