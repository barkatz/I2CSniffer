/*
 * port.h
 *
 *  Created on: Dec 25, 2014
 *      Author: Zig
 */

#ifndef DRIVERS_PORT_H_
#define DRIVERS_PORT_H_

#include "stm32f2xx.h"

#define PORT_RCC_MASKx(_N)            	 (RCC_AHB1Periph_GPIOA << (_N))
#define PORT_PIN_MASK(_N)             	 (1 << (_N))
#define PORT_GPIOx(_N)                	 ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(_N)))

/*
 * A generic GPIO
 */
class Port {
public:
	/*
	 * @GPIO_Port 	- Choose the port for the GPIO: 0->A, 1->B, 2->C...
	 * @GPIO Pin 	- Choose the pin 1,2,3...
	 * @GPIO_Speed 	- Speed...
	 * @GPIO_OType  -
	 * @GPIO_PuPd   -
	 */
	Port(uint32_t GPIO_Port, uint32_t GPIO_Pin, GPIOSpeed_TypeDef GPIO_Speed,
			GPIOMode_TypeDef GPIO_Mode, GPIOOType_TypeDef GPIO_OType,
			GPIOPuPd_TypeDef GPIO_PuPd) :
			m_GPIO_Port(GPIO_Port), m_GPIO_Pin(GPIO_Pin), m_GPIO_Speed(GPIO_Speed),
			m_GPIO_Mode(GPIO_Mode), m_GPIO_OType(GPIO_OType), m_GPIO_PuPd(GPIO_PuPd) {}

	void init();

protected:
	uint32_t m_GPIO_Port;
	uint32_t m_GPIO_Pin;
	GPIOSpeed_TypeDef m_GPIO_Speed;
	GPIOMode_TypeDef m_GPIO_Mode;
	GPIOOType_TypeDef m_GPIO_OType;
	GPIOPuPd_TypeDef m_GPIO_PuPd;
};

/**
 * An input port.
 */
class InPort: public Port {
public:
	/*
	 * @GPIO_Port 	- Choose the port for the GPIO: 0->A, 1->B, 2->C...
	 * @GPIO Pin 	- Choose the pin 1,2,3...
	 * @GPIO_Speed 	- Speed...
	 * @GPIO_OType  -
	 * @GPIO_PuPd   -
	 */
	InPort(uint32_t GPIO_Port, uint32_t GPIO_Pin, GPIOSpeed_TypeDef GPIO_Speed,
			GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd);

	uint8_t read(void);

};

/**
 * An output port.
 */
class OutPort: public Port{
public:
	OutPort(uint32_t GPIO_Port, uint32_t GPIO_Pin, GPIOSpeed_TypeDef GPIO_Speed,
			GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd);

	void write(uint8_t val);
};

#endif /* DRIVERS_PORT_H_ */
