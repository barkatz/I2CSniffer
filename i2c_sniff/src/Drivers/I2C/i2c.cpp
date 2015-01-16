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
		m_state(I2C_START_BIT)	{
	init();
}

void I2CSniffer::init() {
	m_currSdaVal = m_oldSdaVal = m_sdaPort.read();
	m_currSclVal = m_oldSclVal = m_sclPort.read();
}

/*
 * Read lines and extract bits
 */
void I2CSniffer::Update() {
	static BITS bit;
	static bool rise_detected = false;
	static size_t bit_count = 0;

	// Save old values
	m_oldSdaVal = m_currSdaVal;
	m_oldSclVal = m_currSclVal;

	// Read current line values
	m_currSdaVal = m_sdaPort.read();
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
		m_state = I2C_ADDRESS; // Waiting for address...
		bit_count = 0;
		return;
	}

	// Check Stop bit:
	// If scl is still high, and sda is pulled high--> This is the stopbit
	if ( (m_currSclVal == 1) &&
			(m_currSdaVal == 1) && (m_oldSdaVal == 0) ) {
		rise_detected = false;
		m_state = I2C_START_BIT; // Waiting for start bit...
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
	// Capture SCL RISE
	if ((m_currSclVal == 1) && (m_oldSclVal == 0)) {
		// Sample the bit on rise time!
		bit = (BITS) m_currSdaVal;

		// If this is the first ONE bit after the START bit, ruin it!
//		if (bit == ONE_BIT && should_ruin_address) {
//			should_ruin_address = false;
//			// configure sda as output and pull low
//			reconfigure_sda_pin_as_output_and_pull_low();
//
//			// Wait for scl to go down again.
//			do {
//				m_sclVal = m_sclPort.read();
//			} while (m_sclVal);
//
//			// Reconfigure SDA again as input.
//			reconfigure_sda_pin_as_input();
//
//			// TODO: we push the bit here without capturing SCL fall.
//			// I think its ok but need to verify that.
//			m_buffer.push(bit);
//			bitcount++;
//		} else {
			rise_detected = true;
//		}
	}

	// Capture SCL FALL
	if ((m_currSclVal == 0) && (m_oldSclVal == 1)) {
		if (rise_detected) {
			bit_count++;
			m_buffer.push(bit);
			// If we recieved 7 bits of address, we expect for read write
			if (bit_count == 7 && m_state == I2C_ADDRESS) {
				m_state = I2C_READ_WRITE;
				bit_count = 0;
			// If we recieved the Read/Write byte we are waiting in on data.
			} else if (bit_count == 1 && m_state == I2C_READ_WRITE) {
				m_state = I2C_DATA;
				bit_count = 0;
			// If we recieved 8 bits of data we are now waiting on ack.
			} else if (bit_count == 8 && m_state == I2C_DATA) {
				m_state = I2C_DATA;
				bit_count = 0;
			}
			// If we recieved 1 bit of ack, we are waiting for more data :)
			} else if (bit_count == 1 && m_state == I2C_DATA) {
				m_state = I2C_DATA;
				bit_count = 0;
			}


		rise_detected = false;
	}

}



