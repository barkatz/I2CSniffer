#include <stdio.h>
#include <vector>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include "lcd_log.h"
//#include "Drivers\port.h"
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
	MATCH_ADDRES, 		// Match 7 address bits
	READ_WRITE_BIT,		// Get read/write bit
	ACK_ADDRESS_BIT,	// Get Ack bit

	READ_BYTE,			// Master reads 8 bits of data (We are sending)
	READ_BYTE_ACK,		// Read 1 bit of ack/nack from master (We are receiving)

	WRITE_BYTE,			// Master writes 8 bits of data (We are receiving)
	WRITE_BYTE_ACK,		// We write ack (Actually the real slave does ^^)
	WRITE_BYTE_UNK		// In this state we read the first bit of the next data, or stop bit.
};

// Maximum size of read/write buffers.
#define BUFFER_MAX_SIZE 0x200

// Easier macro
#define READ_SDA() 						read_port(SDA_PORT_IN, SDA_PIN_IN)
#define READ_SCL() 						read_port(SCL_PORT_IN, SCL_PIN_IN)
#define WRITE_SDA(val)					write_port(SDA_PORT_OUT, SDA_PIN_OUT, val)

#define CLEAR_PR(pin) 					EXTI->PR = 1 << (pin)

#define CAPTURE_FALLING_EDGE(pin) 	EXTI->RTSR &= ~(1 << (pin)); \
									EXTI->FTSR |= (1 << (pin))

#define CAPTURE_RISING_EDGE(pin) 	EXTI->FTSR &= ~(1 << (pin)); \
									EXTI->RTSR |= (1 << (pin))

#define PUSH_PULL_LINE(port,pin)	PORT_GPIOx(port)->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t) pin)); \
									PORT_GPIOx(port)->OTYPER |= (uint16_t) (GPIO_OType_PP << ((uint16_t) pin))

#define OPEN_DRAIN_LINE(port,pin) 	PORT_GPIOx(port)->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t) pin)); \
									PORT_GPIOx(port)->OTYPER |= (uint16_t) (GPIO_OType_OD	<< ((uint16_t) pin))

/********************************************************************************************************
 *  Globals
 *********************************************************************************************************/
// The prefix of the address to intercept - Always starts with a start bit
BITS address_to_intercept[] = { ZERO_BIT, ZERO_BIT, ZERO_BIT, ONE_BIT, ONE_BIT,	ZERO_BIT, ONE_BIT };
// How many bits already matched.
uint8_t match_index = 0;
// State of the state machine that flips.
FLIP_STATE state;

// Bytes written/read buffers.
uint32_t bytes_read[BUFFER_MAX_SIZE];
uint32_t bread 						= 0;
uint32_t bytes_written[BUFFER_MAX_SIZE];
uint32_t bwrite 					= 0;

// Global counters
uint32_t interception_count 		= 0;
uint32_t miss_count 				= 0;
uint32_t address_nacked 			= 0;
uint32_t missmatched_prefix 		= 0;
uint32_t matched_prefix				= 0;
uint32_t read_address_matches 		= 0;
uint32_t write_address_matches 		= 0;
uint32_t write_byte_nacks 			= 0;

unsigned int tt = 0;
unsigned int vals[][6] = {
		{0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc}, // 0
		{0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb}, // 1
		{0x01, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa}, // 2
		//{0xf1, 0x9f, 0xff, 0x85, 0x01}, // 3 -> 86 deg
		{0x3d, 0x00, 0xb1, 0xff, 0x6f, 0x01}, // 3 -> 86 deg
		//{0x0, 0x00, 0x0, 0x0, 0x0, 0x01}, // 3 -> 86 deg
};


/********************************************************************************************************
 *  Decl
 *********************************************************************************************************/

/*
 * Reads from a port
 */
bool inline read_port(uint32_t port, uint8_t pin);

/*
 * Writes to a port
 */
void inline write_port(uint32_t port, uint8_t pin, bool val);

/*
 * Inits buttons behavior.
 */
void init_buttons();

/*
 * Inits LCD display.
 */
void init_lcd(void);



/*********************************************************************************************************
 *  Impl
 **********************************************************************************************************/
bool inline read_port(uint32_t port, uint8_t pin) {
	return (PORT_GPIOx(port)->IDR & PORT_PIN_MASK(pin)) != 0;
}

void inline write_port(uint32_t port, uint8_t pin, bool val) {
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
 * Interrupts must be defined extern!
 */
#ifdef __cplusplus
extern "C" {
#endif
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

	// Read lines
	bool scl = READ_SCL();
	bool sda = READ_SDA();

	if (!sda && scl) {
		// SDA falls while SCL was high -> start bit.
		// start looking for a prefix match
		match_index = 0;
		state = MATCH_ADDRES;

		// 1) disable sda interrupt.
		mask_interrupt(SDA_PIN_IN);
		// 2) start clock interrupt.
		unmask_interrupt(SCL_PIN_IN);
	}
	// Clear SDA Interrupt.
	CLEAR_PR(SDA_PIN_IN);

}
GPIO_TypeDef* GPIOx = PORT_GPIOx(SDA_PORT_OUT);


void inline scan_start_bit() {
	// 1) disable SCL interrupt
	mask_interrupt(SCL_PIN_IN);
	// 2) enable SDA interrupt (searching for start bit)
	unmask_interrupt(SDA_PIN_IN);
}

/*
 * SCL Interrupt (PA2)
 */
void EXTI2_IRQHandler(void) {
	static volatile uint8_t 	bit_count 			= 0; 		// Bit count for receiving/xmiting a byte
	static volatile uint32_t 	byte_to_recv 		= 0; 		// a temp byte to receive.
	static bool 				is_read 			= 0;		// a flag to indiciate whether master is reading/writing
	static uint32_t 			byte_to_send 		= 0x01;		// A temp byte for sending
	static unsigned int 		last_byte_written	= 0;		// The last byte written
	static uint32_t 	 		read_index 			= 0;		// The read index in a long read transaction
	bool 						stop_bit 			= false;	// Did we hit a stop bit after a write transaction.

	// Read lines
	bool scl = READ_SCL();
	bool sda = READ_SDA();

	switch (state) {
	// Are we reading the address
	case MATCH_ADDRES:
		if (address_to_intercept[match_index] == (BITS) sda) {
			match_index++;
			// in case of a matching bit, check if we have matched the entire address prefix.
			if (match_index == sizeof(address_to_intercept)) {
				// Address captured, move on to capturing read/write
				state = READ_WRITE_BIT;
				matched_prefix++;
			}
		} else {
			missmatched_prefix++;
			scan_start_bit();
		}
		break;

	case READ_WRITE_BIT:
		// Save the read/write op.
		is_read = sda;
		if (is_read) {
			read_address_matches++;
			read_index = 0;
		} else {
			write_address_matches++;
		}
		// Reset the bitcount and temp_byte for the current byte to be transmitted/sent
		bit_count = 0;
		byte_to_recv = 0;
		// And wait for ack for the address.
		state = ACK_ADDRESS_BIT;
		break;

	case ACK_ADDRESS_BIT:
		// Check for NAK/ACK
		if (sda) {
			// NAK
			//-> Address doesn't exist. search for start bit.
			address_nacked++;
			scan_start_bit();
		} else {
			// ACK
			// start receive/send data.
			if (is_read) {
				// Master wishes to read.
				// We want to pull sda line to 0/1. For this, we change to capture SCL fall
				// And we will come back after the ack bit SCL has fallen.
				CAPTURE_FALLING_EDGE(SCL_PIN_IN);
				state = READ_BYTE;
			} else {
				// Sample the byte which is currently being written.
				state = WRITE_BYTE;
			}
		}
		break;

	case READ_BYTE:
		//
		// We get here on the falling edge of the ACK bit of the address.
		// Send 8 bits of data to the master
		//
		if (bit_count < 8) {
			// set SDA OUT port to push/pull...
			if (bit_count == 0) {
				PUSH_PULL_LINE(SDA_PORT_OUT, SDA_PIN_OUT);
				if (last_byte_written == 0x3) {
					byte_to_send = vals[3][read_index++];
				} else  {
					byte_to_send = 0x01;
				}
				if (read_index >5) {
					read_index = 0;
					byte_to_send = 0x1;
					vals[3][tt] = vals[3][tt] + 1;
					while (vals[3][tt] == 0xff) {
						tt = (tt + 1) % 4;
					}
					vals[3][tt]++;
					for (unsigned int i=0; i<tt ; i++) {
						vals[3][i] = 0;
					}
					tt = 0;
				}
			}
			// Write the current bit of data
			WRITE_SDA(byte_to_send & (1<<(7-bit_count)));
			bit_count++;
		} else {
			// Finished writing the bits to the master
			// Change back to capture rising edge of SCL, to capture the ACK bit
			CAPTURE_RISING_EDGE(SCL_PIN_IN);

			// Change back sda to open drain.
			OPEN_DRAIN_LINE(SDA_PORT_OUT, SDA_PIN_OUT);

			// Release it...
			WRITE_SDA(1);

			// Update counters and wait for ack.
			bread++;
			state = READ_BYTE_ACK;
		}
		break;

	case READ_BYTE_ACK:
		if (sda) {
			// NACK -> Last byte the master wants to read. We expect stop bit.
			scan_start_bit();
		} else {
			// ACK -> Master wants to read more bytes
			bit_count = 0;
			byte_to_recv = 0;
			// Master wishes to read.
			// We want to pull sda line to 0/1. For this, we change to capture SCL fall
			// And we will come back after the ack bit SCL has fallen.
			CAPTURE_FALLING_EDGE(SCL_PIN_IN);
			state = READ_BYTE;
		}
		break;

	case WRITE_BYTE:
		// Read 8 bits of data
		bit_count++;
		byte_to_recv = (byte_to_recv << 1) | sda;
		if (bit_count == 8) {
			// Save the byte
			bytes_written[bwrite % BUFFER_MAX_SIZE] = byte_to_recv;
			last_byte_written = byte_to_recv;
			bwrite++;
			state = WRITE_BYTE_ACK;
		}
		break;

	case WRITE_BYTE_ACK:
		if (sda) {
			// slave could not process it. nothing we can do
			write_byte_nacks++;
		} else {
			// This is tricky. The master can now do 2 things:
			// A) Send another byte
			// B) Send a stop
			// We now wait for clock to go up again. then we will check if its a data bit or stop bit
			state = WRITE_BYTE_UNK;
		}
		break;

	case WRITE_BYTE_UNK:
			// We captured SCL rise, this might be the next data bit or a stop bit.
			// The easiest way to check this is with a loop....
			mask_interrupt(SCL_PIN_IN);

			// check if sda is changing while scl is still high...
			do {
				if (sda != READ_SDA()) {
					// This means stop bit
					stop_bit = true;
				}
				scl = READ_SCL();
			} while (scl && !stop_bit);

			// if it was a stop bit, start scanning for start bit.
			if (stop_bit) {
				scan_start_bit();
			} else {
				// if it wasn't a stop bit, it was bit 1 of the next byte
				bit_count = 1;
				byte_to_recv = sda;
				state = WRITE_BYTE;
				unmask_interrupt(SCL_PIN_IN);
			}
		break;
	} /* switch (state) */
	CLEAR_PR(SCL_PIN_IN);

}
#ifdef __cplusplus
}
#endif

int main(int argc, char* argv[]) {
	// Sda port is pulled up. Let it go floating.
	init_lcd();
	init_buttons();

	/*
	 * Set up interrupts and gpios.
	 * Set output port as pullup and write '1' (release)
	 * Catch falling edge of SDA (START BIT)
	 * Catch rising edge of SCL (DATA BIT)
	 */
	init_port(SDA_PORT_OUT, SDA_PIN_OUT, GPIO_Speed_50MHz, GPIO_Mode_OUT, GPIO_OType_OD, GPIO_PuPd_UP);
	WRITE_SDA(1);


	init_gpio_port_interrupts_and_connect_to_nvic(SDA_PORT_IN, SDA_PIN_IN,	EXTI_Trigger_Falling);
	init_gpio_port_interrupts_and_connect_to_nvic(SCL_PORT_IN, SCL_PIN_IN,	EXTI_Trigger_Rising);
	// Start looking for start bit.
	unmask_interrupt(SDA_PIN_IN);

	while (1) {	}

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
