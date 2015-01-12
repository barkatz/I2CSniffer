/*
 * i2c.cpp
 *
 *  Created on: Dec 25, 2014
 *      Author: Zig
 */

#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "Drivers\i2c\i2c.h"
#include <string.h>

/*
 *  Ctor:
 *  1) Creates the 2 input ports for sda/scl.
 *  2) Read current values.
 */
I2CSniffer::I2CSniffer(uint32_t SDA_Port, uint32_t SDA_Pin,
		uint32_t SCL_Port, uint32_t SCL_Pin, const size_t buffer_size) :
		m_buffer(buffer_size),
		m_sdaPort(SDA_Port, SDA_Pin, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_NOPULL),
		m_sclPort(SCL_Port, SCL_Pin, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_NOPULL)
		{

	m_sdaVal = m_sdaPort.read();
	m_sclVal = m_sclPort.read();
}


/*
 * Read lines and extract bits
 */
void I2CSniffer::Update() {
	static BITS bit;
	static bool rise_detected = false;
//	static bool should_ruin_address = false;
	// Save old values
	uint32_t  oldsda = m_sdaVal;
	uint32_t  oldscl = m_sclVal;

	// Read current line values
	m_sdaVal = m_sdaPort.read();
	m_sclVal = m_sclPort.read();

	/*
	 *  Checks for stop and start bit ->
	 *  Only 2 bits where SDA changes while SCL is HIGH
	 */
	// Check start bit:
	// If scl is still high, and sda is pulled low --> This is the start bit
	if ( (m_sclVal == 1) &&
			(m_sdaVal == 0) && (oldsda == 1) ) {
		rise_detected = false;
//		should_ruin_address = true;
		m_buffer.push(START_BIT);
		return;
	}

	// Check Stop bit:
	// If scl is still high, and sda is pulled high--> This is the stopbit
	if ( (m_sclVal == 1) &&
			(m_sdaVal == 1) && (oldsda == 0) ) {
		rise_detected = false;
//		should_ruin_address = false;
		m_buffer.push(STOP_BIT);
		return;
	}

	/*
	 * Check regular bits.
	 * Regular bits are transformed in the following way:
	 * SDA is set up when SCL is LOW with the right bit (0/1). When SDA is set to the desired value,
	 * then SCL is freed (i.e pulled up) by the transmitter, and after some time pulled low again to xmit the next bit.
	 * We make sure we see both rising and falling edge to avoid adding the start/stop bits as bits....
	 */
	if ((m_sclVal == 1) && (oldscl == 0)) {
		bit = (BITS) m_sdaVal;
//		// ruin the first address...
//		if (bit == ONE_BIT && should_ruin_address) {
//
//		}
		rise_detected = true;
	}
	if ((m_sclVal == 0) && (oldscl == 1)) {
		if (rise_detected) {
			m_buffer.push(bit);
		}
		rise_detected = false;
	}

}

