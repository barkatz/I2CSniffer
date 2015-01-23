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
 *  The port classes are inline for speed issues!
 */

/*
 * A generic GPIO
 */
class Port {
public:
	/**
	 * @GPIO_Port 	- Choose the port for the GPIO: 0->A, 1->B, 2->C...
	 * @GPIO Pin 	- Choose the pin 1,2,3...
	 * @GPIO_Speed 	- Speed
	 * @GPIO_OType  - Output type
	 * @GPIO_PuPd   -
	 */
	Port(uint32_t GPIO_Port, uint32_t GPIO_Pin, GPIOSpeed_TypeDef GPIO_Speed,
			GPIOMode_TypeDef GPIO_Mode, GPIOOType_TypeDef GPIO_OType,
			GPIOPuPd_TypeDef GPIO_PuPd) :	m_GPIO_Port(GPIO_Port), 	m_GPIO_Pin(GPIO_Pin) {
		// Turn on the clock for the port
		RCC_AHB1PeriphClockCmd(PORT_RCC_MASKx(m_GPIO_Port), ENABLE);

		// Configure pin in output push/pull mode
		m_GPIO_InitStructure.GPIO_Pin 			= PORT_PIN_MASK(m_GPIO_Pin);
		m_GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed;
		m_GPIO_InitStructure.GPIO_Mode 			= GPIO_Mode;
		m_GPIO_InitStructure.GPIO_OType 		= GPIO_OType;
		m_GPIO_InitStructure.GPIO_PuPd 			= GPIO_PuPd;
		GPIO_Init(PORT_GPIOx(m_GPIO_Port), &m_GPIO_InitStructure);
	}

	/**
	 * Sets the port in a disabled state. This can be usefull when a port has
	 * 2 configurations (input/output) and used in InOutPort.
	 */

	inline uint32_t getPort() {
		return m_GPIO_Port;
	}

	inline uint32_t getPin() {
		return m_GPIO_Pin;
	}


protected:
	// All of the ports charastristics.
	uint32_t 				m_GPIO_Port; 			// GPIO Port 0->A, 1->B, 2-->C ....
	uint32_t 				m_GPIO_Pin; 			// The pin 1,2,3...
	GPIO_InitTypeDef 		m_GPIO_InitStructure; 	// Port charastristics
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
			GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd):
			Port(GPIO_Port, GPIO_Pin, GPIO_Speed, GPIO_Mode_IN, GPIO_OType, GPIO_PuPd) {}

	inline uint8_t read(void) {
		// If this port is enabled simply read a bit.
		return GPIO_ReadInputDataBit(PORT_GPIOx(m_GPIO_Port), (uint16_t) PORT_PIN_MASK(m_GPIO_Pin));
	}
};

/**
 * An output port.
 */
class OutPort: public Port {
public:
	/*
	 * @GPIO_Port 	- Choose the port for the GPIO: 0->A, 1->B, 2->C...
	 * @GPIO Pin 	- Choose the pin 1,2,3...
	 * @GPIO_Speed 	- Speed...
	 * @GPIO_OType  -
	 * @GPIO_PuPd   -
	 */
	OutPort(uint32_t GPIO_Port, uint32_t GPIO_Pin, GPIOSpeed_TypeDef GPIO_Speed,
			GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd) :
				Port(GPIO_Port, GPIO_Pin, GPIO_Speed, GPIO_Mode_OUT, GPIO_OType,GPIO_PuPd) {}

	inline void write(uint8_t val) {
		// Simply write a bit
		GPIO_WriteBit(PORT_GPIOx(m_GPIO_Port), (uint16_t) PORT_PIN_MASK(m_GPIO_Pin), val? Bit_SET: Bit_RESET);
	}
};


#endif /* DRIVERS_PORT_H_*/

