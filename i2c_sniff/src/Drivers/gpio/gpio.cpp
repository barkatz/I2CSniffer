/*
 * gpio.c
 *
 *  Created on: Jan 30, 2015
 *      Author: Zig
 */


#include "stm32f2xx.h"
#include "Drivers/port.h"
#include "Drivers/gpio/gpio.hpp"

/*
 * Configures Port.Pin as INPUT.
 * Connects the port to the NVIC
 * At first all events are masked!
 */
void init_gpio_port_interrupts_and_connect_to_nvic(uint8_t port, uint8_t pin, EXTITrigger_TypeDef trigger) {
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint8_t PIN_TO_EXTIX_IRQ[] = {
			  EXTI0_IRQn, // 0
			  EXTI1_IRQn, // 1
			  EXTI2_IRQn, // 2
			  EXTI3_IRQn, // 3
			  EXTI4_IRQn, // 4
			  EXTI9_5_IRQn, // 5
			  EXTI9_5_IRQn,  //6
			  EXTI9_5_IRQn,  //7
			  EXTI9_5_IRQn,  //8
			  EXTI9_5_IRQn,  //9
			  EXTI15_10_IRQn, // 10
			  EXTI15_10_IRQn, // 11
			  EXTI15_10_IRQn, // 12
			  EXTI15_10_IRQn, // 13
			  EXTI15_10_IRQn, // 14
			  EXTI15_10_IRQn  // 15

	};
	/* Enable the GPIO clock for the port */
	RCC_AHB1PeriphClockCmd(PORT_RCC_MASKx(port), ENABLE);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure Pin.Port pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = PORT_PIN_MASK(pin);
	GPIO_Init(PORT_GPIOx(port), &GPIO_InitStructure);

	/* Connect EXTI to the port. Note that PA.i/PB.i... are connected to line_i */
	SYSCFG_EXTILineConfig(port, pin);


	/* Configure EXTI linei to capture trigger evernts */
	EXTI_InitStructure.EXTI_Line = 1<<pin ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = trigger;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	/* Mask interrupts! */
	EXTI->IMR &= ~EXTI_InitStructure.EXTI_Line;

	/* Enable and set EXTI Line2 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = PIN_TO_EXTIX_IRQ[pin];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}





