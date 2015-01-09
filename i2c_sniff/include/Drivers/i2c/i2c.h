/*
 * i2c.h
 *
 *  Created on: Dec 25, 2014
 *      Author: Zig
 */

#ifndef DRIVERS_I2C_I2C_H_
#define DRIVERS_I2C_I2C_H_
#include "port.h"

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
				uint32_t SCL_Port, uint32_t SCL_Pin);

	bool Update();
protected:
	InPort m_sdaPort;
	InPort m_sclPort;

};

#endif /* DRIVERS_PORT_H_ */






#endif /* DRIVERS_I2C_I2C_H_ */
