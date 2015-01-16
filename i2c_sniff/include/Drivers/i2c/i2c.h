/*
 * i2c.h
 *
 *  Created on: Dec 25, 2014
 *      Author: Zig
 */

#ifndef DRIVERS_I2C_I2C_H_
#define DRIVERS_I2C_I2C_H_
#include "../port.h"
#include "../../CyclicBuffer.h"


enum BITS {
	ZERO_BIT = 0,
	ONE_BIT = 1,
	START_BIT = 2,
	STOP_BIT = 3,
};

enum I2C_STATE{
	I2C_START_BIT 			= 0,
	I2C_ADDRESS   			= 1,
	I2C_READ_WRITE			= 2,
	I2C_ADDR_ACK			= 3,
	I2C_DATA  				= 4,
	I2C_DATA_ACK			= 5,
	//I2C_STOP_BIT 			= 6,
};
/*
 * A generic GPIO
 */
class I2CSniffer {
public:
	/*
	 * @SDA_Port 		- Choose the port for SDA: 0->A, 1->B, 2->C...
	 * @GPIO SDA_Pin 	- Choose the pin  for SDA: 1,2,3...
	 * @SCL_Port 		- Choose the port for SCL: 0->A, 1->B, 2->C...
	 * @SCL_Pin  		- Choose the port for SCL: 0->A, 1->B, 2->C...
	 */
	I2CSniffer(uint32_t SDA_Port, uint32_t SDA_Pin,
				uint32_t SCL_Port, uint32_t SCL_Pin, const size_t m_size);

	void init();
	/**
	 * Samples the SDA/SCL lines and update the buffer with new bits
	 */
	void Update();

	CyclicBuffer<BITS> m_buffer;
protected:

	InOutPort m_sdaPort;
	InOutPort m_sclPort;
	uint32_t m_currSdaVal;
	uint32_t m_currSclVal;
	uint32_t m_oldSdaVal;
	uint32_t m_oldSclVal;
	I2C_STATE m_state;


};


#endif /* DRIVERS_I2C_I2C_H_ */
