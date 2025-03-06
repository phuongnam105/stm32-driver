/*
 * stm32f405xx_rcc_driver.h
 *
 *  Created on: Mar 3, 2025
 *      Author: Phuong Nam
 */

#ifndef INC_STM32F405XX_RCC_DRIVER_H_
#define INC_STM32F405XX_RCC_DRIVER_H_

#include "stm32f405xx.h"

uint32_t RCC_GetPCLK1Value();
uint32_t RCC_GetPCLK2Value();
uint32_t RCC_GetPLLOutputClock();

#endif /* INC_STM32F405XX_RCC_DRIVER_H_ */
