/*
 * i2c.hpp
 *
 *  Created on: Jan 30, 2015
 *      Author: Zig
 */

#ifndef DRIVERS_I2C_I2C_HPP_
#define DRIVERS_I2C_I2C_HPP_



// I2C PINS
// I2C1_SCL --> PB.6 (CN4/13)
// I2C1_SDA --> PB.9 (CN4/8)
/* I2Cx Communication boards Interface */
#define I2Cx                          I2C1
#define I2Cx_CLK                      RCC_APB1Periph_I2C1
#define I2Cx_EV_IRQn                  I2C1_EV_IRQn
#define I2Cx_ER_IRQn                  I2C1_ER_IRQn
#define I2Cx_EV_IRQHANDLER            I2C1_EV_IRQHandler
#define I2Cx_ER_IRQHANDLER            I2C1_ER_IRQHandler

#define I2Cx_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2Cx_SDA_PIN                  GPIO_Pin_9
#define I2Cx_SDA_GPIO_PORT            GPIOB
#define I2Cx_SDA_SOURCE               GPIO_PinSource9
#define I2Cx_SDA_AF                   GPIO_AF_I2C1

#define I2Cx_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2Cx_SCL_PIN                  GPIO_Pin_6
#define I2Cx_SCL_GPIO_PORT            GPIOB
#define I2Cx_SCL_SOURCE               GPIO_PinSource6
#define I2Cx_SCL_AF                   GPIO_AF_I2C1

// Fast i2c
#define I2C_SPEED 340000
#define I2C_DUTYCYCLE I2C_DutyCycle_16_9
//#define I2C_SPEED 100000
//#define I2C_DUTYCYCLE  I2C_DutyCycle_2
//#define SLAVE_ADDRESS 0x80
//#define SLAVE_ADDRESS (0x12)
//#define SLAVE_ADDRESS (0x18)
//#define SLAVE_ADDRESS (0x0)
#define SLAVE_ADDRESS (0x10)



void i2c_init(void);

#endif /* DRIVERS_I2C_I2C_HPP_ */
