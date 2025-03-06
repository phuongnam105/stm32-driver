/*
 * stm32f405xx_usart_driver.c
 *
 *  Created on: Feb 27, 2025
 *      Author: Phuong Nam
 */

#include "stm32f405xx_usart_driver.h"

void USART_PeriClockControl(USART_RegDef_t* pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
			USART1_PCLK_EN();

		if (pUSARTx == USART2)
			USART2_PCLK_EN();

		if (pUSARTx == USART3)
			USART3_PCLK_EN();

		if (pUSARTx == UART4)
			UART4_PCLK_EN();

		if (pUSARTx == UART5)
			UART5_PCLK_EN();

		if (pUSARTx == USART6)
			USART6_PCLK_EN();
	}
	else{
		if (pUSARTx == USART1)
			USART1_PCLK_DI();

		if (pUSARTx == USART2)
			USART2_PCLK_DI();

		if (pUSARTx == USART3)
			USART3_PCLK_DI();

		if (pUSARTx == UART4)
			UART4_PCLK_DI();

		if (pUSARTx == UART5)
			UART5_PCLK_DI();

		if (pUSARTx == USART6)
			USART6_PCLK_DI();
	}
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pUSARTx->USART_CR1 |= ( 1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_UE);
	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if (pUSARTx->USART_SR & (1 << FlagName))
	{
		return SET;
	}

	return RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->USART_SR &= ~(StatusFlagName);
}

void USART_SetBaudRate (USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t xPCLK;

	uint32_t usartdiv;

	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	if (pUSARTx == USART1 || pUSARTx == USART6)
	{
		xPCLK = RCC_GetPCLK2Value();
	}
	else
	{
		xPCLK == RCC_GetPCLK1Value();
	}

	if(pUSARTx->USART_CR1 & ( 1 << USART_CR1_OVER8))
	{
		usartdiv = ((25 * xPCLK) / (2* BaudRate));
	}
	else
	{
		usartdiv = ((25 * xPCLK) / (4 * BaudRate));
	}

	M_part = usartdiv /100;

	tempreg |= M_part << 4;

	F_part = usartdiv - (M_part*100);

	if(pUSARTx ->USART_CR1 & ( 1 << USART_CR1_OVER8))
	{
		F_part = ((( F_part * 8) + 50)/100) & ((uint8_t)0x07);
	}
	else
	{
		F_part = ((( F_part * 16) + 50)/100) & ((uint8_t)0x0F);
	}

	tempreg |= F_part;

	pUSARTx->USART_BRR = tempreg;
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg = 0 ;

	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	/////////////////////////////Configuraion of CR1///////////////////////////////////////////

	//Implement to configure mode ( Tx or RX, TxRx)
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg |= ( 1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= ( 1 << USART_CR1_TE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE);
	}

	//Implement to configure wordlength
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	//Implement to configure of parity control bit fields
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE );
	}
	else if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		//enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);
		//enable the odd parity
		tempreg |= ( 1 << USART_CR1_PS);
	}

	pUSARTHandle->pUSARTx->USART_CR1 = tempreg;

	/////////////////////////////////Configuration of CR2///////////////////////////////////////

	tempreg = 0;

	//Implement the code to configure the number of stop bits

	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	pUSARTHandle->pUSARTx->USART_CR2 = tempreg;

	/////////////////////////////////Configuration of CR3///////////////////////////////////////

	tempreg =0;

	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= ( 1 << USART_CR3_RTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE) | ( 1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->USART_CR3 = tempreg;

	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	for (uint32_t i =0 ; i < Len ; i++)
	{
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TXE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9 bit, load the DR with 2 bytes masking the bits other than first 9 bit
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this tranfer.So, 9bit of user data will be sent
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//parity bit is used in this tranfer.So, 8bit of user data will be sent
				//9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//this is 8bit data transfer
			pUSARTHandle->pUSARTx->USART_DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	for(uint32_t i =0 ; i < Len ; i++)
	{
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_RXNE));

		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used.So, all 9 bits will be of user data
				//read only first 9 bit.So, mask the DR with 0x01FF
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->USART_DR & (uint16_t)0x01FF);

				pRxBuffer++;
			}
			else
			{
				//parity is used.So , 8bit will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);

				pRxBuffer++;
			}
		}
		else
		{
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used.So, all 8 bits will be of user data
				*pRxBuffer = pUSARTHandle->pUSARTx->USART_DR;

			}
			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				*pRxBuffer = (uint8_t)pUSARTHandle->pUSARTx->USART_DR;
			}

			pRxBuffer++;
		}
	}
}
