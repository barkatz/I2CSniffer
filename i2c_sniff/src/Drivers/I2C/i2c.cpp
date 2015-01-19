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
#include <stdio.h>
/*
 *  Ctor:
 *  1) Creates the 2 input ports for sda/scl.
 *  2) Read current values.
 */

//uint32_t GPIO_Port,
//			uint32_t GPIO_Pin,
//			GPIOSpeed_TypeDef GPIO_OutSpeed,
//			GPIOOType_TypeDef GPIO_OutOType,
//			GPIOPuPd_TypeDef GPIO_OutPuPd,
//			GPIOPuPd_TypeDef GPIO_InPuPd
I2CSniffer::I2CSniffer(uint32_t SDA_Port, uint32_t SDA_Pin,	uint32_t SCL_Port, uint32_t SCL_Pin, const size_t buffer_size) :
		m_buffer(buffer_size),
		m_sdaPort(	SDA_Port,
					SDA_Pin,
					GPIO_Speed_50MHz,
					GPIO_OType_OD,
					GPIO_PuPd_NOPULL,
					GPIO_PuPd_NOPULL
					),
		m_sclPort(SCL_Port,
					SCL_Pin,
					GPIO_Speed_50MHz,
					GPIO_OType_OD,
					GPIO_PuPd_NOPULL,
					GPIO_PuPd_NOPULL),
					m_flipNextOneBit(false) {
	init();
}

void I2CSniffer::init() {
	m_currSdaVal = m_oldSdaVal = m_sdaPort.read();
	m_currSclVal = m_oldSclVal = m_sclPort.read();
}

/*
 * Read lines and extract bits
 */
bool I2CSniffer::Update() {
	static BITS bit;
	static bool rise_detected = false;
	static size_t bit_count = 0;


	// Save old values of sda/scl
	m_oldSdaVal = m_currSdaVal;
	m_oldSclVal = m_currSclVal;

	// Read current line values
	// Before pooling the lines, don't poll sda line if we are in the middle of writing 0 on it...
	// This can happen if we are flipping the next ONE_BIT to zero when intercepting address bit and ACK,
	// Or when transmitting a byte on the I2C when we act as the slave
	if (m_flipNextOneBit) {
		m_currSdaVal = 1; 											// In this case the original bit is one, and we are flipping it to zero
	} else if (m_writingByteBitIndex) {
		m_currSdaVal = (m_writingByte >> m_writingByteBitIndex)&1; // In this case the sda value is the current bit we transmitting
	} else {
		m_currSdaVal = m_sdaPort.read();							// In this case we are reading the bit ^^
	}

	// The clock is always being read since the master controls it.
	m_currSclVal = m_sclPort.read();

	/*
	 *  Checks for stop and start bit ->
	 *  Only 2 bits where SDA changes while SCL is HIGH
	 */
	// Check start bit:
	// If scl is still high, and sda is pulled low --> This is the start bit
	if ( (m_currSclVal == 1) &&
			(m_currSdaVal == 0) && (m_oldSdaVal == 1) ) {
		rise_detected = false;
		m_buffer.push(START_BIT);
		bit_count = 0;
		return true;
	}

	// Check Stop bit:
	// If scl is still high, and sda is pulled high--> This is the stopbit
	if ( (m_currSclVal == 1) &&
			(m_currSdaVal == 1) && (m_oldSdaVal == 0) ) {
		rise_detected = false;
		m_buffer.push(STOP_BIT);
		return true;
	}

	/*
	 * Check regular bits.
	 * Regular bits are transformed in the following way:
	 * SDA is set up when SCL is LOW with the right bit (0/1). When SDA is set to the desired value,
	 * then SCL is freed (i.e pulled up) by the transmitter, and after some time pulled low again to xmit the next bit.
	 * We make sure we see both rising and falling edge to avoid adding the start/stop bits as bits....
	 */
	// Capture SCL RISE
	if ((m_currSclVal == 1) && (m_oldSclVal == 0)) {
		// Sample the bit on rise time!
		bit = (BITS) m_currSdaVal;
		rise_detected = true;
	}

	// Capture SCL FALL
	if ((m_currSclVal == 0) && (m_oldSclVal == 1)) {
		bool retval = false;
		if (rise_detected) {
			bit_count++;
			retval = true;
			m_buffer.push(bit);
			rise_detected = false;
			// If we were flipping a bit - Stop
			if (m_flipNextOneBit) {
				m_flipNextOneBit = false;
			}

			// If we are writing the bytes, prepare SDA with the next byte.
			if (m_writingByteBitIndex) {
				// First advance to the next bit
				m_writingByteBitIndex--;
				m_sdaPort.write((m_writingByte >> m_writingByteBitIndex)&1); // We loaded sda val with the bit to transmit
			}
		}
		return retval;
	}

	return false;
}

void I2CSniffer::flip_next_one_bit() {
	m_flipNextOneBit = true;
	m_sdaPort.write(0);
}


void I2CSniffer::writeByte(uint8_t byte) {
	m_writingByteBitIndex = 7;
	m_writingByte = byte;
	// Prepare the first bit
	m_sdaPort.write((byte >> 7) & 0x1);
	//m_sda
}
