#include <stdio.h>
#include <vector>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include "lcd_log.h"
#include "Drivers\port.h"
#include "Drivers/i2c/i2c.hpp"
#include "Drivers/gpio/gpio.hpp"
#include "utils.hpp"
#include "stm32f2xx.h"

//#include "stm32fxx_exti.h"

// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f2xx.c
//

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wunused-variable"

/**************************************************
 * Enums & Defines
 ***************************************************/
enum FLIP_STATE {
	SCAN_FOR_START,
	MATCH_PREFIX,
	FLIP_BIT_START,
	FLIP_BIT_END
};
/********************************************************************************************************
 *  Globals
 *********************************************************************************************************/

InPort sdaPortIn(SDA_PORT_IN, SDA_PIN_IN, GPIO_Speed_50MHz, GPIO_OType_OD,
		GPIO_PuPd_NOPULL);

OutPort sdaPortOut(SDA_PORT_OUT, SDA_PIN_OUT, GPIO_Speed_50MHz, GPIO_OType_OD,
		GPIO_PuPd_UP);

InPort sclPortIn(SCL_PORT_IN, SCL_PIN_IN, GPIO_Speed_50MHz, GPIO_OType_OD,
		GPIO_PuPd_NOPULL);

/* This boolean flag used to flag scl_rise event.
 * We sample SDA when SCL falls, and treat it as a data bit.
 * This logic must check if there was an scl rise event before (not if for example, there was a start/end bit).
 */
//static bool scl_rise = false;
// The prefix of the address to intercept - Always starts with a start bit
//BITS address_to_intercept[] = { ONE_BIT, ZERO_BIT, ZERO_BIT }; // for 0xd0
// For 0xd/0xc --> 0b0000110X
BITS address_to_intercept[] = { ZERO_BIT , ZERO_BIT, ZERO_BIT, ONE_BIT };
// How many bits already matched.
uint8_t match_index = 0;
// State of the state machine that flips.
FLIP_STATE state;


uint32_t interception_count = 0;
uint32_t miss_count = 0;

uint32_t reg_index = 0;
uint8_t regs[] = 	{	0,	 	0,		0,	 	0xc4, 	0xff, 	0xae, 	0xff, 	0x6c, // 0x0 	- 	0x7
						0x01,	0,	 	0,	 	0, 		0, 		0,		0,		0,	  // 0x8 	- 	0xF
						0,		0, 		0, 		0, 		0, 		0,		0,		0,	  // 0x10 	-	0x17
						0,		0, 	 	0, 		0, 		0, 		0,		0,		0,	  // 0x18 	-	0x1f
						0,		0, 	 	0, 		0, 		0, 		0,		0,		0,	  // 0x20 	-	0x27
						0,		0, 	 	0, 		0, 		0, 		0,		0,		0,
						0,		0, 	 	0, 		0, 		0, 		0,		0,		0,
						0,		0, 	 	0, 		0, 		0, 		0,		0,		0 };


/********************************************************************************************************
 *  Decl
 *********************************************************************************************************/

void init_buttons();

/*********************************************************************************************************
 *  Impl
 **********************************************************************************************************/
bool inline read(uint32_t port, uint8_t pin) {
	return (PORT_GPIOx(port)->IDR & PORT_PIN_MASK(pin)) != 0;
}

void inline write(uint32_t port, uint8_t pin, bool val) {
	if (val != 0) {
		PORT_GPIOx(port)->BSRRL = (uint16_t) PORT_PIN_MASK(pin);
	} else {
		PORT_GPIOx(port)->BSRRH = (uint16_t) PORT_PIN_MASK(pin);
	}

}
void init_lcd(void) {
	// Init lcd log.
	STM322xG_LCD_Init();
	LCD_LOG_Init();
	LCD_LOG_SetHeader((uint8_t*) "BARS I2C Sniffer");
}

void init_buttons() {
	// Init the button interrupt
	STM_EVAL_PBInit(BUTTON_KEY, BUTTON_MODE_EXTI);
}

/**
 * Interrupts must be as extern!
 */
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief  This function handles I2Cx Error interrupt request.
 * @param  None
 * @retval None
 */
void I2Cx_ER_IRQHANDLER(void) {
	/* Read SR1 register to get I2C error */
	if ((I2C_ReadRegister(I2Cx, I2C_Register_SR1) & 0xFF00) != 0x00) {
		/* Clears error flags */
		I2Cx->SR1 &= 0x00FF;
	}

}

struct i2c_event {
uint32_t event;
uint8_t dr;
};

i2c_event volatile events[0x300];
uint32_t z = 0;


uint32_t sent_bytes = 0;
/**
 * @brief  This function handles I2Cx event interrupt request.
 * @param  None
 * @retval None
 */
void I2Cx_EV_IRQHANDLER(void) {
	__IO uint32_t Event = 0x00;
	/* Get Last I2C Event */
	Event = I2C_GetLastEvent(I2Cx);
	uint8_t dr =  (uint8_t)I2Cx->DR;


	if (z >= sizeof(events)/sizeof(i2c_event)) {
		mask_interrupt(SDA_PIN_IN);
		LCD_BarLog("Cant record anymore events\n");
		return;
	}
	events[z].event = Event;
	events[z].dr = dr;
	z++;

	// If address matched -> search for start bit!
	if ((I2C_SR1_ADDR | I2C_EVENT_SLAVE_STOP_DETECTED) & Event)  {
//		if (interception_count<0x50) {
			unmask_interrupt(SDA_PIN_IN);
			interception_count++;
//		} else {
//			LCD_BarLog("Done\n");
//		}
	}
	switch (Event) {
	/* ****************************************************************************/
	/*                          Slave Transmitter Events                          */
	/* ****************************************************************************/
	case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
//		LCD_BarLog("Transmitter (Reading...) address matched\n");
		I2C_SendData(I2Cx, regs[reg_index++]);
		sent_bytes++;
		//I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
		break;

	case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
	case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
		//LCD_BarLog("Xmitting a byte to master\n");
		I2C_SendData(I2Cx, regs[reg_index++]);
		break;

	/* ****************************************************************************/
	/*                              Slave Receiver Events                         */
	/* ****************************************************************************/
	/* Master is trying to write to us... 0x00020002(131074)
	 * This means our address with write bit was matched.						  */
	case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
		break;

	/*  Address matched (0x2) and DR contains data (0x40). */
	case 0x20042:
		reg_index = dr;
		break;
	/* 0x200 */
	case I2C_EVENT_SLAVE_BYTE_RECEIVED:
	case (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_SR1_BTF):
		regs[reg_index++] = dr;
		break;


	case I2C_EVENT_SLAVE_STOP_DETECTED:
		break;

	default:
		break;
	}
	reg_index = reg_index % sizeof(regs);
}
/*
 *  Button handler (KEY button is IRQ-15)
 *  This prints the buffer of the sniffed I2C msgs to the screen.
 */
void EXTI15_10_IRQHandler(void) {
	EXTI_ClearFlag(EXTI_Line15);
	init_lcd();
}

/**
 * SDA Interrupt (PC0)
 * This is interrupt is ON only when scanning for the start bit!
 */
void EXTI0_IRQHandler(void) {
	EXTI->PR = 1<<SDA_PIN_IN;

	bool scl = read(SCL_PORT_IN, SCL_PIN_IN);
	if (scl) {
		// SDA falls while SCL was high -> start bit.
		// start looking for a prefix match
		match_index = 0;
		state = MATCH_PREFIX;

		// 1) disable sda interrupt.
		mask_interrupt(SDA_PIN_IN);
		// 2) start clock interrupt.
		unmask_interrupt(SCL_PIN_IN);



	}
}
GPIO_TypeDef* GPIOx  = PORT_GPIOx(SDA_PORT_OUT);
//uint32_t pinpos 	= SDA_PIN_IN;
/*
 * SCL Interrupt (PA2)
 */
void EXTI2_IRQHandler(void) {
	EXTI->PR = 1<<SCL_PIN_IN;
	bool scl_risen = false;
	bool sda = read(SDA_PORT_IN, SDA_PIN_IN);
	// Are we currently fliping a bit?
	switch (state) {
		case SCAN_FOR_START:
		break;
		case MATCH_PREFIX:
			if (address_to_intercept[match_index] == (BITS) sda) {
				match_index++;
				// in case of a matching bit, check if we have matched the entire address prefix.
				if (match_index == sizeof(address_to_intercept)) {
					// We need to capture SCL clock fall now - and flip the next bit
					EXTI->RTSR &= ~(1 << SCL_PIN_IN);
					EXTI->FTSR |= (1 << SCL_PIN_IN);
					state = FLIP_BIT_START;
				}
			} else {
				// 1) disable SCL interrupt
				mask_interrupt(SCL_PIN_IN);
				// 2) enable SDA interrupt (searching for start bit)
				unmask_interrupt(SDA_PIN_IN);
			}
			break;
		case FLIP_BIT_START:
			// Pull SDA low and wait for next scl fall.
//			for (int i=0; i<23; i++) {
//				write(SDA_PORT_OUT, SDA_PIN_OUT, 1);
//			}
			write(SDA_PORT_OUT, SDA_PIN_OUT, 0);

			state = FLIP_BIT_END;
			break;

		case FLIP_BIT_END:
			// Realse SDA. We are done ! This works like shit
//		    GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)SDA_PIN_OUT * 2));
//		    GPIOx->PUPDR |= (GPIO_PuPd_NOPULL << (SDA_PIN_OUT * 2));
//	        /* Output mode configuration*/
//	        GPIOx->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)SDA_PIN_OUT)) ;
//	        GPIOx->OTYPER |= (uint16_t)(GPIO_OType_PP << ((uint16_t)SDA_PIN_OUT));
//		    for (int i=0; i<15; i++) {
		    	write(SDA_PORT_OUT, SDA_PIN_OUT, 1);
//		    }
//		    GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)SDA_PIN_OUT * 2));
//		    GPIOx->PUPDR |= (GPIO_PuPd_UP << (SDA_PIN_OUT * 2));
//	        GPIOx->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)SDA_PIN_OUT)) ;
//	        GPIOx->OTYPER |= (uint16_t)(GPIO_OType_OD << ((uint16_t)SDA_PIN_OUT));


			// Restore to interrupts to capture scl raise (for data bits)
			EXTI->RTSR |= (1 << SCL_PIN_IN);
			EXTI->FTSR &= ~(1 << SCL_PIN_IN);
			// 1) disable SCL interrupt
			mask_interrupt(SCL_PIN_IN);
			// 2) enable SDA interrupt (searching for start bit)
			// Will be enabled in i2c :)
//			unmask_interrupt(SDA_PIN_IN);
			break;
		}
}
#ifdef __cplusplus
}
#endif


int main(int argc, char* argv[]) {
	// Sda port is pulled up. Let it go floating.
	sdaPortOut.write(1);
	init_lcd();
	init_buttons();
	i2c_init();

	/*
	 * Set up interrupts.
	 * Catch falling edge of SDA (START BIT)
	 * Catch rising edge of SCL (DATA BIT)
	 */
	init_gpio_port_interrupts_and_connect_to_nvic(SDA_PORT_IN, SDA_PIN_IN, EXTI_Trigger_Falling);
	init_gpio_port_interrupts_and_connect_to_nvic(SCL_PORT_IN, SCL_PIN_IN, EXTI_Trigger_Rising);
	// Start looking for start bit.
	unmask_interrupt(SDA_PIN_IN);

	while (1) {

	}

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
