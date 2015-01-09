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
void Port::init() {
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
}

/**************************************************
 * Input port
 **************************************************/
InPort::InPort(uint32_t GPIO_Port, uint32_t GPIO_Pin , GPIOSpeed_TypeDef GPIO_Speed,
		GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd) :
		Port(GPIO_Port, GPIO_Pin, GPIO_Speed, GPIO_Mode_IN, GPIO_OType,GPIO_PuPd) {
	Port::init();
}

uint8_t InPort::read(void) {
	return GPIO_ReadInputDataBit(PORT_GPIOx(m_GPIO_Port), (uint16_t) PORT_PIN_MASK(m_GPIO_Pin));
}

/**************************************************
 * Output port
 **************************************************/
OutPort::OutPort(uint32_t GPIO_Port, uint32_t GPIO_Pin , GPIOSpeed_TypeDef GPIO_Speed,
		GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd) :
		Port(GPIO_Port, GPIO_Pin, GPIO_Speed, GPIO_Mode_OUT, GPIO_OType,GPIO_PuPd) {
	Port::init();
}

void OutPort::write(uint8_t val) {
	GPIO_WriteBit(PORT_GPIOx(m_GPIO_Port), (uint16_t) PORT_PIN_MASK(m_GPIO_Pin), val? Bit_SET: Bit_RESET);
}


