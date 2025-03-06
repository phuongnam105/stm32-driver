/*
 * stm32f405xx.h
 *
 *  Created on: Jan 9, 2025
 *      Author: Phuong Nam
 */

#ifndef INC_STM32F405XX_H_
#define INC_STM32F405XX_H_


#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/*
 * ARM Cortex Mx Processor NVIC ISERx register Address
 */

#define NVIC_ISER0 			( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1			( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2			( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3			( (__vo uint32_t*)0xE000E10C )
#define NVIC_ISER4			( (__vo uint32_t*)0xE000E110 )
#define NVIC_ISER5			( (__vo uint32_t*)0xE000E114 )
#define NVIC_ISER6			( (__vo uint32_t*)0xE000E118 )
#define NVIC_ISER7			( (__vo uint32_t*)0xE000E11C )

/*
 * ARM Cortex Mx Processor NVIC ICERx register Address
 */
#define NVIC_ICER0 			( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1			( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2			( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3			( (__vo uint32_t*)0xE000E18C )
#define NVIC_ICER4			( (__vo uint32_t*)0xE000E190 )
#define NVIC_ICER5			( (__vo uint32_t*)0xE000E194 )
#define NVIC_ICER6			( (__vo uint32_t*)0xE000E198 )
#define NVIC_ICER7			( (__vo uint32_t*)0xE000E19C )

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 *
 */
#define NO_PR_BITS_IMPLEMENTED	4

/* base address of  memories */

#define FLASH_BASEADDR 						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define SRAM2_BASEADDR						0x2001C000U
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM								SRAM1_BASEADDR


/* base address of bus domain */

#define AHB1_BASEADDR 						0x40020000U
#define APB1_BASEADDR						0x40000000U
#define APB2_BASEADDR						0x40010000U
#define PERIPH_BASEADDR						0x40000000U

/*base address of peripheral */

/* AHB1 */

#define GPIOA_BASEADDR						( 0x40020000U + AHB1_BASEADDR )
#define GPIOB_BASEADDR						( 0x40020400U + AHB1_BASEADDR )
#define GPIOC_BASEADDR						( 0x40020800U + AHB1_BASEADDR )
#define GPIOD_BASEADDR						( 0x40020C00U + AHB1_BASEADDR )
#define GPIOE_BASEADDR						( 0x40021000U + AHB1_BASEADDR )
#define GPIOF_BASEADDR						( 0x40021400U + AHB1_BASEADDR )
#define GPIOG_BASEADDR						( 0x40021800U + AHB1_BASEADDR )
#define GPIOH_BASEADDR						( 0x40021C00U + AHB1_BASEADDR )
#define GPIOI_BASEADDR						( 0x40022000U + AHB1_BASEADDR )
#define RCC_BASEADDR						( 0x40023800U + AHB1_BASEADDR )

/* APB1 */

#define I2C1_BASEADDR  						( 0x40005400U + APB1_BASEADDR )
#define I2C2_BASEADDR 						( 0x40005800U + APB1_BASEADDR )
#define I2C3_BASEADDR 						( 0x40005C00U + APB1_BASEADDR )
#define SPI2_BASEADDR 						( 0x40003800U + APB1_BASEADDR )
#define SPI3_BASEADDR 						( 0x40003C00U + APB1_BASEADDR )
#define USART2_BASEADDR 					( 0x40004400U + APB1_BASEADDR )
#define USART3_BASEADDR 					( 0x40004800U + APB1_BASEADDR )
#define UART4_BASEADDR 						( 0x40004C00U + APB1_BASEADDR )
#define UART5_BASEADDR 						( 0x40005000U + APB1_BASEADDR )

/* APB2 */

#define SPI1_BASEADDR 						( 0x40013000U + APB2_BASEADDR )
#define USART1_BASEADDR						( 0x40011000U + APB2_BASEADDR )
#define USART6_BASEADDR 					( 0x40011400U + APB2_BASEADDR )
#define EXTI_BASEADDR 						( 0x40013C00U + APB2_BASEADDR )
#define SYSCFG_BASEADDR 					( 0x40013800U + APB2_BASEADDR )

/*
 * Peripheral register definition for GPIO
 */

typedef struct
{
	__vo uint32_t MODER;  	/* GPIO port mode register 					Address offset: 0x00 */
	__vo uint32_t OTYPER;	/* GPIO port output type register 			Address offset: 0x04 */
	__vo uint32_t OSPEEDR;	/* GPIO port output speed register  		Address offset: 0x08 */
	__vo uint32_t PUPDR;	/* GPIO port pull-up/pull-down register		Address offset: 0x0C */
	__vo uint32_t IDR;		/* GPIO port input data register 			Address offset: 0x10 */
	__vo uint32_t ODR;		/* GPIO port output data register 			Address offset: 0x14 */
	__vo uint32_t BSRR;		/* GPIO port bit set/reset register			Address offset: 0x18 */
	__vo uint32_t LCKR;		/* GPIO port configuration lock register 	Address offset: 0x1C */
	__vo uint32_t AF[2];	/* GPIO alternate function low/high register 	Address offset: 0x20 */

} GPIO_RegDef_t;


/*
 * Peripheral definition (type casted to xxx_RegDef_t)
 */

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t*)GPIOI_BASEADDR)

/*
 * Peripheral register definition for RCC
 */

typedef struct
{
	__vo uint32_t CR; 					/* Address offset  0x00*/
	__vo uint32_t PLLCFGR;				/* Address offset  0x04*/
	__vo uint32_t CFGR; 				/* Address offset 0x08*/
	__vo uint32_t CIR; 					/* Address offset 0x0C*/
	__vo uint32_t AHB1RSTR; 			/* Address offset 0x10*/
	__vo uint32_t AHB2RSTR; 			/* Address offset 0x14*/
	__vo uint32_t AHB3RSTR; 			/* Address offset 0x18*/
	uint32_t Reserved1;					/* Address offset 0x1C*/
	__vo uint32_t APB1RSTR; 			/* Address offset 0x20*/
	__vo uint32_t APB2RSTR;			 	/* Address offset 0x24*/
	uint32_t Reserved2[2];				/* Address offset 0x28->0x2C*/
	__vo uint32_t AHB1ENR; 				/* Address offset 0x30*/
	__vo uint32_t AHB2ENR;				/* Address offset  0x34*/
	__vo uint32_t AHB3ENR; 				/* Address offset 0x38*/
	uint32_t Reserved3;					/* Address offset 0x3C*/
	__vo uint32_t APB1ENR; 				/* Address offset 0x40*/
	__vo uint32_t APB2ENR; 				/* Address offset 0x44*/
	uint32_t Reserved4[2];				/* Address offset 0x48->0x4C*/
	__vo uint32_t AHB1LPENR;			/* Address offset 0x50*/
	__vo uint32_t AHB2LPENR;			/* Address offset 0x54*/
	__vo uint32_t AHB3LPENR;			/* Address offset 0x58*/
	uint32_t Reserved5;					/* Address offset 0x5C*/
	__vo uint32_t APB1LPENR;			/* Address offset 0x60*/
	__vo uint32_t APB2LPENR;			/* Address offset 0x64*/
	uint32_t Reserved6[2];				/* Address offset 0x68->0x6C*/
	__vo uint32_t BDCR;					/* Address offset 0x70*/
	__vo uint32_t CSR;					/* Address offset 0x74*/
	uint32_t Reserved7[2];				/* Address offset 0x78->0x7C*/
	__vo uint32_t SSCGR;				/* Address offset 0x80*/
	__vo uint32_t PLLI2SCFGR;			/* Address offset 0x84*/
} RCC_RegDef_t;

/*
 * RCC definition
 */

#define RCC	((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR; 		/*!< address offset: 0x00 */
	__vo uint32_t EMR;		/*!< address offset: 0x04 */
	__vo uint32_t RTSR;		/*!< address offset: 0x08 */
	__vo uint32_t FTSR;		/*!< address offset: 0x0C */
	__vo uint32_t SWIER;	/*!< address offset: 0x10 */
	__vo uint32_t PR;		/*!< address offset: 0x14 */
}EXTI_RegDef_t;

/*
 * EXTI definition
 */

#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 * peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP; 		/*!< address offset 0x00 */
	__vo uint32_t PMC;			/*!< address offset 0x04 */
	__vo uint32_t EXTICR[4];	/*!< address offset 0x08-0x14 */
	uint32_t Reserved1[2];		/*!< address offset 0x018-0x1C */
	__vo uint32_t CMPCR;		/*!< address offset 0x20 */
	uint32_t Reserved2[2];		/*!< address offset 0x24-0x28 */
	__vo uint32_t CFRG;			/*!< address offset 0x2C */
}SYSCFG_RegDef_t;

/*
 * SYSCFG definition
 */

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
/*
 * peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;
}SPI_RegDef_t;

/*
 * SPI definition
 */

#define SPI1 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASEADDR)

/*
 * peripheral register definition structure for I2C
 */

typedef struct
{
	__vo uint32_t I2C_CR1;
	__vo uint32_t I2C_CR2;
	__vo uint32_t I2C_OAR1;
	__vo uint32_t I2C_OAR2;
	__vo uint32_t I2C_DR;
	__vo uint32_t I2C_SR1;
	__vo uint32_t I2C_SR2;
	__vo uint32_t I2C_CCR;
	__vo uint32_t I2C_TRISE;
	__vo uint32_t I2C_FLTR;
}I2C_RegDef_t;

/*
 * I2C definition
 */

#define I2C1	((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3	((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * peripheral register definition structure for USART
 */

typedef struct
{
	__vo uint32_t USART_SR;
	__vo uint32_t USART_DR;
	__vo uint32_t USART_BRR;
	__vo uint32_t USART_CR1;
	__vo uint32_t USART_CR2;
	__vo uint32_t USART_CR3;
	__vo uint32_t USART_GTPR;
}USART_RegDef_t;

/*
 * USART definition
 */

#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3		((USART_RegDef_t*)USART3_BASEADDR)
#define USART6		((USART_RegDef_t*)USART6_BASEADDR)

/*
 * UART definition
 */

#define UART4		((USART_RegDef_t*)UART4_BASEADDR)
#define UART5		((USART_RegDef_t*)UART5_BASEADDR)

/*
 * returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 : \
										(x == GPIOB) ? 1 : \
										(x == GPIOC) ? 2 : \
										(x == GPIOD) ? 3 : \
										(x == GPIOE) ? 4 : \
										(x == GPIOF) ? 5 : \
										(x == GPIOG) ? 6 : \
										(x == GPIOH) ? 7 : \
										(x == GPIOI) ? 8 : 0 )

/*
 * Clock Enable Macros for GPIOx peripheral
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= ( 1<<0 ))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= ( 1<<1 ))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= ( 1<<2 ))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= ( 1<<3 ))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= ( 1<<4 ))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= ( 1<<5 ))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= ( 1<<6 ))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= ( 1<<7 ))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= ( 1<<8 ))

/*
 * Clock Enable Macros for UART peripheral
 */

#define UART4_PCLK_EN()	(RCC->APB1ENR |= ( 1<<19 ))
#define UART5_PCLK_EN()	(RCC->APB1ENR |= ( 1<<20 ))

/*
 * Clock Enable Macros for USART peripheral
 */

#define USART1_PCLK_EN() (RCC->APB2ENR |= ( 1<<4 ))
#define USART6_PCLK_EN() (RCC->APB2ENR |= ( 1<<5 ))
#define USART2_PCLK_EN() (RCC->APB1ENR |= ( 1<<17 ))
#define USART3_PCLK_EN() (RCC->APB1ENR |= ( 1<<18 ))

/*
 * Clock Enable Macros for I2C peripheral
 */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= ( 1<<21 ))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= ( 1<<22 ))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= ( 1<<23 ))

/*
 * Clock Enable Macros for SPI peripheral
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= ( 1<<12 ))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= ( 1<<14 ))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= ( 1<<15 ))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= ( 1<<14))

/*
 * Clock Disable Macros for GPIOx peripheral
 */

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~( 1<<0 ))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~( 1<<1 ))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~( 1<<2 ))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~( 1<<3 ))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~( 1<<4 ))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~( 1<<5 ))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~( 1<<6 ))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~( 1<<7 ))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~( 1<<8 ))

/*
 * Clock Disable Macros for UART peripheral
 */

#define UART4_PCLK_DI()	(RCC->APB1ENR &= ~( 1<<19 ))
#define UART5_PCLK_DI()	(RCC->APB1ENR &= ~( 1<<20 ))


/*
 * Clock Disable Macros for USART peripheral
 */

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~( 1<<4 ))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~( 1<<5 ))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~( 1<<17 ))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~( 1<<18 ))

/*
 * Clock Disable Macros for I2C peripheral
 */

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~( 1<<21 ))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~( 1<<22 ))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~( 1<<23 ))

/*
 * Clock Disable Macros for SPI peripheral
 */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~( 1<<12 ))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~( 1<<14 ))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~( 1<<15 ))

/*
 * Clock Disable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~( 1<<14))
/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)


/*
 * Generic macros
 */

#define SET 			1
#define RESET	 		0
#define ENABLE 			SET
#define DISABLE 		RESET
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_RESET     	RESET
#define FLAG_SET 	   	SET

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     	31
#define IRQ_NO_I2C1_ER     	32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of i2c peripheral
 ******************************************************************************************/

/*
 * Bit position definition I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_SMBUS					1
#define I2C_CR1_SMBTYPE					3
#define I2C_CR1_ENARP					4
#define I2C_CR1_ENPEC					5
#define I2C_CR1_ENGC					6
#define I2C_CR1_NOSTRETCH				7
#define I2C_CR1_START					8
#define I2C_CR1_STOP					9
#define I2C_CR1_ACK						10
#define I2C_CR1_POS						11
#define I2C_CR1_PEC						12
#define I2C_CR1_ALERT					13
#define I2C_CR1_SWRST					15

/*
 * Bit position definition I2C_CR2
 */
#define I2C_CR2_FREQ					0
#define I2C_CR2_ITERREN					8
#define I2C_CR2_ITEVTEN					9
#define I2C_CR2_ITBUFEN					10
#define I2C_CR1_DMAEN					11
#define I2C_CR1_LAST					12

/*
 * Bit position definition I2C_SR1
 */
#define I2C_SR1_SB						0
#define I2C_SR1_ADDR					1
#define I2C_SR1_BTF						2
#define I2C_SR1_ADD10					3
#define I2C_SR1_STOPF					4
#define I2C_SR1_RxNE					6
#define I2C_SR1_TxE						7
#define I2C_SR1_BERR					8
#define I2C_SR1_ARLO					9
#define I2C_SR1_AF						10
#define I2C_SR1_OVR						11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT					14
#define I2C_SR1_SMBALERT				15

/*
 * Bit position definition I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY					1
#define I2C_SR2_TRA						2
#define I2C_SR2_GENCALL					4
#define I2C_SR2_SMBDEFAULT				5
#define I2C_SR2_SMBHOST					6
#define I2C_SR2_DUALF					7
#define I2C_SR2_PEC						8

/*
 * Bit position definition I2C_CCR
 */
#define I2C_CCR_CCR						0
#define I2C_CCR_DUTY					14
#define I2C_CCR_FS						15

/******************************************************************************************
 *Bit position definitions of usart peripheral
 ******************************************************************************************/

/*
 * Bit position definition USART_CR1
 */
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

/*
 * Bit position definition USART_CR2
 */
#define USART_CR2_ADD	0
#define USART_CR2_LBDL	5
#define USART_CR2_LBDIE	6
#define USART_CR2_LBCL	8
#define USART_CR2_CPHA	9
#define USART_CR2_CPOL	10
#define USART_CR2_CLKEN	11
#define USART_CR2_STOP	12
#define USART_CR2_LINEN	14

/*
 * Bit position definition USART_CR3
 */
#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11

/*
 * Bit position definition USART_SR
 */
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

/*
 * Bit position definition USART_BRR
 */
#define USART_BRR_DIV_FRACTION	0
#define USART_BRR_DIV_MANTISSA	4


/*
 * Bit position definition USART_GTPR
 */
#define USART_GTPR_PSC		0
#define USART_GTPR_GT		8

#include "stm32f405xx_gpio_driver.h"
#include "stm32f405xx_spi_driver.h"
#include "stm32f405xx_i2c_driver.h"
#include "stm32f405xx_usart_driver.h"
#include "stm32f405xx_rcc_driver.h"


#endif /* INC_STM32F405XX_H_ */
