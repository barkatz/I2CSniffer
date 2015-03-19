/*
 * gpio.hpp
 *
 *  Created on: Jan 30, 2015
 *      Author: Zig
 */

#ifndef DRIVERS_GPIO_GPIO_HPP_
#define DRIVERS_GPIO_GPIO_HPP_
/*-------------------------------------------------------
 * PIN OUT
 * ------------------------------------------------------
 * IMPORTANT NOTE
 * ~~~~~~~~~~~~~~
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

// PC.10 (CN4/36) --> Uart Tx port
#define UARTX_PORT_TX		2
#define UARTX_PIN_TX		10

// PC.11 (CN4/35) --> Uart Rx port
#define UARTX_PORT_RX		2
#define UARTX_PIN_RX		11



#define PORT_RCC_MASKx(_N)            	 (RCC_AHB1Periph_GPIOA << (_N))
#define PORT_PIN_MASK(_N)             	 (1 << (_N))
#define PORT_GPIOx(_N)                	 ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(_N)))

/*************************************************
 * Inline functions (performance sensitive)
 *************************************************/

void inline unmask_interrupt(uint8_t pin) {
	EXTI->PR |= (1 << pin);
	EXTI->IMR |= (1 << pin);
	EXTI->PR |= (1 << pin);
}

void inline mask_interrupt(uint8_t pin) {
	EXTI->IMR &= ~(1 << pin);
}


/*************************************************
 * Other functions
 *************************************************/
/*
 * Configures Port.Pin as INPUT, and connects it to NVIC, waiting for 'trigger' interrupts.
 */
void init_gpio_port_interrupts_and_connect_to_nvic(uint8_t port, uint8_t pin,	EXTITrigger_TypeDef trigger);

/*
 * Init a gpio port.
 */
void init_port(uint32_t GPIO_Port, uint32_t GPIO_Pin, GPIOSpeed_TypeDef GPIO_Speed,
		GPIOMode_TypeDef GPIO_Mode, GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd);

#endif /* DRIVERS_GPIO_GPIO_HPP_ */
