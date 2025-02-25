/*
 * 007i2c_master_receive.c
 *
 *  Created on: Feb 19, 2025
 *      Author: Phuong Nam
 */

#include "stm32f405xx_i2c_driver.h"
#include <string.h>

#define MY_ADDR 	0x61

I2C_Handle_t I2C1Handle;


void GPIO_I2C_Init(void)
{
	GPIO_Handle_t I2CPin;

	I2CPin.pGPIOx = GPIOB;
	I2CPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	I2CPin.GPIO_PinConfig.GPIO_PinAltFunMode = 4;

	//SDA
	I2CPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPin);

	//SCL
	I2CPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPin);
}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress =	MY_ADDR;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;

	I2C_Init(&I2C1Handle);
}

int main(void)
{
	GPIO_I2C_Init();
	I2C1_Inits();

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, ENABLE);
}
