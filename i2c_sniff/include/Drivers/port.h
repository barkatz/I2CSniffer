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
	/**
	 * @GPIO_Port 	- Choose the port for the GPIO: 0->A, 1->B, 2->C...
	 * @GPIO Pin 	- Choose the pin 1,2,3...
	 * @GPIO_Speed 	- Speed
	 * @GPIO_OType  - Output type
	 * @GPIO_PuPd   -
	 */
	Port(uint32_t GPIO_Port, uint32_t GPIO_Pin, GPIOSpeed_TypeDef GPIO_Speed,
			GPIOMode_TypeDef GPIO_Mode, GPIOOType_TypeDef GPIO_OType,
			GPIOPuPd_TypeDef GPIO_PuPd, bool isEnabled) :
			m_GPIO_Port(GPIO_Port),
			m_GPIO_Pin(GPIO_Pin),
			m_GPIO_Speed(GPIO_Speed),
			m_GPIO_Mode(GPIO_Mode),
			m_GPIO_OType(GPIO_OType),
			m_GPIO_PuPd(GPIO_PuPd),
			m_isEnabled(isEnabled) {}

	/**
	 * enables the port with the current configuration.
	 */
	void enable();

	/**
	 * Sets the port in a disabled state. This can be usefull when a port has
	 * 2 configurations (input/output) and used in InOutPort.
	 */
	void disable();


	bool getEnabled();
	uint32_t getPort();
	uint32_t getPin();
protected:
	// All of the ports charastristics.
	uint32_t 				m_GPIO_Port; 		// GPIO Port 0->A, 1->B, 2-->C ....
	uint32_t 				m_GPIO_Pin; 		// The pin 1,2,3...
	GPIOSpeed_TypeDef 		m_GPIO_Speed; 		// The speed of the port relevant only for Outputs (2MHZ/10MHZ/50MHZ...)
	GPIOMode_TypeDef 		m_GPIO_Mode; 		// The mode (in/out)
	GPIOOType_TypeDef 		m_GPIO_OType; 		// Type of output pushpull/opendrain (This is for output only)
	GPIOPuPd_TypeDef 		m_GPIO_PuPd; 		// type of pull (up/down/none)

	bool 					m_isEnabled;		// Whether the port is currently disabled (need to call init to enable)
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
			GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd, bool isEnabled);

	uint8_t read(void);

};

/**
 * An output port.
 */
class OutPort: public Port{
public:
	/*
	 * @GPIO_Port 	- Choose the port for the GPIO: 0->A, 1->B, 2->C...
	 * @GPIO Pin 	- Choose the pin 1,2,3...
	 * @GPIO_Speed 	- Speed...
	 * @GPIO_OType  -
	 * @GPIO_PuPd   -
	 */
	OutPort(uint32_t GPIO_Port, uint32_t GPIO_Pin, GPIOSpeed_TypeDef GPIO_Speed,
			GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd, bool isEnabled);

	void write(uint8_t val);
};

/*
 * A port that can be either written to and read from
 */
class InOutPort {
public:
	/*
	 * @GPIO_Port 	- Choose the port for the GPIO: 0->A, 1->B, 2->C...
	 * @GPIO Pin 	- Choose the pin 1,2,3...
	 * @GPIO_OutSpeed 	- Speed for output port
	 * 					  There's no need for speed for input port
	 * @GPIO_OutOType   - Push/Pull or open drain.
	 * 					 Ther'es no need for OutType for input port

	 * @GPIO_OutPuPd    -  pull for output port
	 * @GPIO_InPuPd     -  pull for input port
	 */
	InOutPort(uint32_t GPIO_Port,
			uint32_t GPIO_Pin,
			GPIOSpeed_TypeDef GPIO_OutSpeed,
			GPIOOType_TypeDef GPIO_OutOType,
			GPIOPuPd_TypeDef GPIO_OutPuPd,
			GPIOPuPd_TypeDef GPIO_InPuPd);


	void write(uint8_t val);
	uint8_t read(void);

protected:
	enum STATE {
		READ,
		WRITE
	};

	InPort		m_inPort;
	OutPort 	m_outPort;
	STATE 		m_state;
};
#endif /* DRIVERS_PORT_H_ */
