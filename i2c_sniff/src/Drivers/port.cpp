/*
 * port.cpp
 *
 *  Created on: Dec 25, 2014
 *      Author: Zig
 */
#include "Drivers/port.h"
#include "stm32f2xx_rcc.h"

/**************************************************
 * Generic port
 **************************************************/
void Port::enable() {
	// Enable GPIO Peripheral clock
	  RCC_AHB1PeriphClockCmd(PORT_RCC_MASKx(m_GPIO_Port), ENABLE);

	  GPIO_InitTypeDef GPIO_InitStructure;

	  // Configure pin in output push/pull mode
	  GPIO_InitStructure.GPIO_Pin = PORT_PIN_MASK(m_GPIO_Pin);
	  GPIO_InitStructure.GPIO_Speed = m_GPIO_Speed;
	  GPIO_InitStructure.GPIO_Mode = m_GPIO_Mode;
	  GPIO_InitStructure.GPIO_OType = m_GPIO_OType;
	  GPIO_InitStructure.GPIO_PuPd = m_GPIO_PuPd;
	  GPIO_Init(PORT_GPIOx(m_GPIO_Port), &GPIO_InitStructure);
	  m_isEnabled = true;
}

void Port::disable() {
	m_isEnabled = false;
}

bool Port::getEnabled() {
	return m_isEnabled;
}

uint32_t Port::getPort() {
	return m_GPIO_Port;
}
uint32_t Port::getPin() {
	return m_GPIO_Pin;
}

/**************************************************
 * Input port
 **************************************************/
InPort::InPort(uint32_t GPIO_Port, uint32_t GPIO_Pin , GPIOSpeed_TypeDef GPIO_Speed,
		GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd, bool isEnabled) :
		Port(GPIO_Port, GPIO_Pin, GPIO_Speed, GPIO_Mode_IN, GPIO_OType, GPIO_PuPd, isEnabled) {
	// If the port was started in enable state-> Configure it as such.
	if (m_isEnabled) {
		Port::enable();
	}
}

uint8_t InPort::read(void) {
	// If this port is enabled simply read a bit.
	if (m_isEnabled) {
		return GPIO_ReadInputDataBit(PORT_GPIOx(m_GPIO_Port), (uint16_t) PORT_PIN_MASK(m_GPIO_Pin));
	} else {
		// Not enabled? enable this port and read a bit
		Port::enable();
		return GPIO_ReadInputDataBit(PORT_GPIOx(m_GPIO_Port), (uint16_t) PORT_PIN_MASK(m_GPIO_Pin));
	}
}

/**************************************************
 * Output port
 **************************************************/
OutPort::OutPort(uint32_t GPIO_Port, uint32_t GPIO_Pin , GPIOSpeed_TypeDef GPIO_Speed,
		GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd, bool isEnabled) :
		Port(GPIO_Port, GPIO_Pin, GPIO_Speed, GPIO_Mode_OUT, GPIO_OType,GPIO_PuPd, isEnabled) {
	if (isEnabled){
		Port::enable();
	}
}

void OutPort::write(uint8_t val) {
	// If the port is enabled -> Simply write a bit
	if (m_isEnabled) {
		GPIO_WriteBit(PORT_GPIOx(m_GPIO_Port), (uint16_t) PORT_PIN_MASK(m_GPIO_Pin), val? Bit_SET: Bit_RESET);
	} else {
		// Not enabled? enable this port and then write a bit
		Port::enable();
		GPIO_WriteBit(PORT_GPIOx(m_GPIO_Port), (uint16_t) PORT_PIN_MASK(m_GPIO_Pin), val? Bit_SET: Bit_RESET);
	}

}


/**************************************************
 * InOut Port
 **************************************************/
InOutPort::InOutPort(uint32_t GPIO_Port,
		uint32_t GPIO_Pin,
		GPIOSpeed_TypeDef GPIO_OutSpeed,
		GPIOOType_TypeDef GPIO_OutOType,
		GPIOPuPd_TypeDef GPIO_OutPuPd,
		GPIOPuPd_TypeDef GPIO_InPuPd):
		 // Remember, OType should be opendrain for input, and Speed is not relevant for inputs, port starts in input state.
		m_inPort(GPIO_Port, GPIO_Pin , GPIO_Speed_50MHz, GPIO_OType_OD,  GPIO_InPuPd, true),
		m_outPort(GPIO_Port, GPIO_Pin , GPIO_OutSpeed, GPIO_OutOType,  GPIO_OutPuPd, false),
		m_state(READ)	{}



void InOutPort::write(uint8_t val) {
	m_outPort.write(val);
}

uint8_t InOutPort::read() {
	return m_inPort.read();
}

