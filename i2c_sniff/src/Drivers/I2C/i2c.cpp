/*
 * i2c.cpp
 *
 *  Created on: Dec 25, 2014
 *      Author: Zig
 */

#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "Drivers\i2c\i2c.h"

/*---------Private Defines...  ------- */
#define SDA_BIT 1
#define SDC_BIT 15
/*---------Private declerations...  ------- */
void _init_gpios(void);


void i2c_init(void) {
	_init_gpios();
}

void _init_gpios(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOA clock */
	__GPIOA_CLK_ENABLE();

	/* Configure PA0 pin as input floating */
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin 	= SDA_BIT; // Pin 0 selected

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable GPIOA clock */
	__GPIOA_CLK_ENABLE();

	/* Configure PA0 pin as input floating */
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin 	= SDC_BIT; // Pin 0 selected

	GPIO_Init(GPIOG, &GPIO_InitStructure);
}

uint8_t read_sda(void) {
	return GPIO_ReadInputDataBit(GPIOA, SDA_BIT);
}

uint8_t read_sdc(void) {
	return GPIO_ReadInputDataBit(GPIOG, SDC_BIT);
}
