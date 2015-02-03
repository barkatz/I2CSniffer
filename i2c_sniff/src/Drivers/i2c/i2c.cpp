/*
 * i2c.cpp
 *
 *  Created on: Jan 30, 2015
 *      Author: Zig
 */

#include "stm32f2xx.h"
#include "Drivers/i2c/i2c.hpp"



/**
 * @brief  Enables the I2C Clock and configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void i2c_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	I2C_DeInit(I2Cx);
    I2C_SoftwareResetCmd(I2Cx, ENABLE);
    I2C_SoftwareResetCmd(I2Cx, DISABLE);
	/* RCC Configuration */
	/*I2C Peripheral clock enable */
	RCC_APB1PeriphClockCmd(I2Cx_CLK, ENABLE);

	/*SDA GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2Cx_SDA_GPIO_CLK, ENABLE);

	/*SCL GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2Cx_SCL_GPIO_CLK, ENABLE);

	/* Reset I2Cx IP */
	RCC_APB1PeriphResetCmd(I2Cx_CLK, ENABLE);

	/* Release reset signal of I2Cx IP */
	RCC_APB1PeriphResetCmd(I2Cx_CLK, DISABLE);

	/* GPIO Configuration */
	/*Configure I2C SCL pin */
	GPIO_InitStructure.GPIO_Pin = I2Cx_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);

	/*Configure I2C SDA pin */
	GPIO_InitStructure.GPIO_Pin = I2Cx_SDA_PIN;
	GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);

	/* Connect PXx to I2C_SCL */
	GPIO_PinAFConfig(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_SOURCE, I2Cx_SCL_AF);

	/* Connect PXx to I2C_SDA */
	GPIO_PinAFConfig(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_SOURCE, I2Cx_SDA_AF);

	/* NVIC configuration */
	/* Configure the Priority Group to 1 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Configure the I2C event priority */
	NVIC_InitStructure.NVIC_IRQChannel = I2Cx_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure I2C error interrupt to have the higher priority */
	NVIC_InitStructure.NVIC_IRQChannel = I2Cx_ER_IRQn;
	NVIC_Init(&NVIC_InitStructure);



	/* Initialize I2C peripheral */
	/*!< I2C Init */

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
	I2C_InitStructure.I2C_OwnAddress1 = SLAVE_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx, &I2C_InitStructure);
	/* Enable Error Interrupt */
	I2C_ITConfig(I2Cx, (I2C_IT_ERR | I2C_IT_EVT /*| I2C_IT_BUF*/), ENABLE);
	/* I2C ENABLE */
	I2C_Cmd(I2Cx, ENABLE);
}

