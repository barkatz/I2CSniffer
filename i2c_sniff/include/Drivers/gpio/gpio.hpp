/*
 * gpio.hpp
 *
 *  Created on: Jan 30, 2015
 *      Author: Zig
 */

#ifndef DRIVERS_GPIO_GPIO_HPP_
#define DRIVERS_GPIO_GPIO_HPP_

/*-------------------------------------------------------
 * IMPORTANT NOTE
 * ------------------------------------------------------
 * SDA/SCL Ports were chosen with extreme care since some ports are pulling the line to 0.
 * Yes. Even when configured as NO PULL, the lines where held down low.
 * I wrote down all the ports i have tried just for documentation, maybe i can figure out later on what
 * happened on those ports..
 * SDA port is - YELLOW -
 * SCL port is - WHITE -
 * //>>> Port PA.2 (CN1/37) - <<<<< THIS WORKS
 * //	 Port PA.3 (CN2/3) 	-  Pull to 0...
 * // 	 Port PA.13 (CN3/8) -  SWDIO no good don't use it (JTAG)
 * //	 Port PF.0 (CN1/15) -  Screen related... Not good...
 * // 	 Port PF.7 (CN1/23) -  Pull to 0 :(
 * //>>> Port PC.0 (CN1/29) - <<<< THIS WORKS
 * //>>> Port PE.2 (CN1/2) -  <<<< THIS WORKS
 */
// PC.0 (CN1/29) - SDA (YELLOW) ---> IN
#define SDA_PORT_IN     2
#define SDA_PIN_IN		0

// PE.2 (CN1/2) - SDA (YELLOW) ---> OUT
#define SDA_PORT_OUT    4
#define SDA_PIN_OUT		2

// PA.2 (CN1/37) - SCL (WITE)  ----> IN
#define SCL_PORT_IN		0
#define SCL_PIN_IN		2

/*
 * Configures Port.Pin as INPUT, and connects it to NVIC, waiting for 'trigger' interrupts.
 */
void init_gpio_port_interrupts_and_connect_to_nvic(uint8_t port, uint8_t pin,
		EXTITrigger_TypeDef trigger);

void inline unmask_interrupt(uint8_t pin) {
	EXTI->PR |= (1 << pin);
	EXTI->IMR |= (1 << pin);
	EXTI->PR |= (1 << pin);
}

void inline mask_interrupt(uint8_t pin) {
	EXTI->IMR &= ~(1 << pin);
}

void inline change_trigger(uint8_t pin, EXTITrigger_TypeDef trigger) {
	uint32_t tmp;
	if (trigger == EXTI_Trigger_Rising_Falling) {
		EXTI->RTSR |= (1 << pin);
		EXTI->FTSR |= (1 << pin);
	} else {
		tmp = (uint32_t) EXTI_BASE;
		tmp += trigger;

	*(__IO uint32_t *) tmp |= (1<<pin);
}
}

//void inline set_port(uint32_t port, uint8_t pin, GPIOMode_TypeDef GPIO_Mode) {
//	uint32_t bit_pin = PORT_PIN_MASK(pin);
//	GPIO_TypeDef * GPIOx = PORT_GPIOx(port);
//
//	GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (bit_pin * 2));
//	GPIOx->MODER |=	(((uint32_t) GPIO_Mode) << (bit_pin * 2));
//
//	if (GPIO_Mode == GPIO_Mode_OUT) {
//
//			/* Speed mode configuration */
//			GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
//			GPIOx->OSPEEDR |= ((uint32_t) (GPIO_InitStruct->GPIO_Speed)
//					<< (pinpos * 2));
//
//			/* Check Output mode parameters */
//			assert_param(IS_GPIO_OTYPE(GPIO_InitStruct->GPIO_OType));
//
//			/* Output mode configuration*/
//			GPIOx->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t) pinpos));
//			GPIOx->OTYPER |=
//					(uint16_t) (((uint16_t) GPIO_InitStruct->GPIO_OType)
//							<< ((uint16_t) pinpos));
//		}
//
//		/* Pull-up Pull down resistor configuration*/
//		GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t) pinpos * 2));
//		GPIOx->PUPDR |=
//				(((uint32_t) GPIO_InitStruct->GPIO_PuPd) << (pinpos * 2));
//	}
//}
//}
#endif /* DRIVERS_GPIO_GPIO_HPP_ */
