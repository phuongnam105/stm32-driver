/*
 * i2c_sending_data.c
 *
 *  Created on: Feb 18, 2025
 *      Author: Phuong Nam
 */

#include "stm32f405xx_i2c_driver.h"
#include <string.h>

#define MY_ADDR 	0x60
#define SLAVE_ADDR	0x64

I2C_Handle_t I2C1Handle;

uint8_t data[] = "My name is Phuong Nam";

void I2C_GPIO_Init(void)
{
	GPIO_Handle_t I2CPin;

	I2CPin.pGPIOx = GPIOB;
	I2CPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPin.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

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
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x60;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;

	I2C_Init(&I2C1Handle);
}

int main(void)
{
	I2C_GPIO_Init();
	I2C1_Inits();

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_MasterSendData(&I2C1Handle, data, strlen((char*)data), SLAVE_ADDR);
}
