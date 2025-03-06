/*
 * stm32f405xx_rcc_driver.c
 *
 *  Created on: Mar 3, 2025
 *      Author: Phuong Nam
 */

#include "stm32f405xx_rcc_driver.h"

uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB_Prescaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value (void)
{
	uint32_t pclk1, SystemClock;

	uint8_t clksrc, ahbp, temp,abp1;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clksrc == 0)
	{
		SystemClock = 16000000;
	}
	else if (clksrc == 1)
	{
		SystemClock = 8000000;
	}
	else
	{
		SystemClock = RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_Prescaler[temp - 8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4)
	{
		abp1 = 1;
	}
	else
	{
		abp1 = APB_Prescaler[temp - 4];
	}

	pclk1 = ((SystemClock / ahbp) / abp1);

	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, SystemClock;

	uint8_t clksrc, ahbp, abp2p, temp;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clksrc == 0)
	{
		SystemClock = 8000000;
	}
	else if (clksrc == 1)
	{
		SystemClock = 16000000;
	}
	else
	{
		RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_Prescaler[temp - 8];
	}

	temp = ((RCC->CFGR >> 13) & 0x7);

	if (temp < 4)
	{
		abp2p = 1;
	}
	else
	{
		abp2p = APB_Prescaler[temp -4 ];
	}

	pclk2 = ((SystemClock / ahbp) / abp2p);

	return pclk2;
}

uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}
