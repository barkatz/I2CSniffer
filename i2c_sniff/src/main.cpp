#include <stdio.h>
#include <vector>
#include <string.h>
#include <stdarg.h>
#include "Timer.h"
#include "BlinkLed.h"
#include "lcd_log.h"
#include "Drivers\port.h"
#include "Drivers\i2c\i2c.h"
#include "stm32f2xx.h"
//#include "stm32fxx_exti.h"


// PC.0 (CN1/29) - SDA (YELLOW)
#define SDA_PORT    2
#define SDA_PIN		0

// PA.2 (CN1/37) - SCL (BLUE)
#define SCL_PORT	0
#define SCL_PIN		2


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
/**************************************************
 * Enums & Defines
 ***************************************************/
enum I2C_STATE{
	I2C_NOT_SYNCED,
	I2C_START_BIT,
	I2C_ADDRESS,
	I2C_STOP_BIT
};

/********************************************************************************************************
 *  Globals
 *********************************************************************************************************/
// Global i2c sniffer.
/*--------------------
 * IMPORTANT NOTE
 * --------------------
 * This inits sda and scl ports as inputs.
 * Those has been chosen with extreme care since some ports are pulling the line to 0.
 * Yes. Even when configured as NO PULL, the lines where held down low.
 * I wrote down all the ports i have tried just for documentation, maybe i can figure out later on what
 * happened on those ports...
 * // Port PA.2 (CN1/37) - SCL (BLUE) 							<---- THIS WORKS
 * // Port PA.3 (CN2/3) - SDA (YELLOW) Pull to 0...
 * // Port PA.13 (CN3/8) - SDA (YELLOW) --> SWDIO no good don't use it (JTAG)
 * // Port PF.0 (CN1/15) - SDA (a-0,b-1,c-2,d-3,e-4,f-5) --> Screen related... Not good...
 * // Port PF.7 (CN1/23) - SDA (a-0,b-1,c-2,d-3,e-4,f-5) --> Pull to 0 :(
 * // Port PC.0 (CN1/29) - SDA (a-0,b-1,c-2,d-3,e-4,f-5) --> < ----- THIS WORKS
 */
I2CSniffer i2c(SDA_PORT, SDA_PIN, SCL_PORT, SCL_PIN, 0x1000);
// A buffer to hold the bits captured from the i2c sniffer for actual prodcessing
BITS unprocessed_bits[0x1000];

/*
 * The address mask of the slave we are targeting.
 * This mask will be 'anded' with the current address of the slave and check if it matches the target.
 * If they match we will flip the next '1' bit.
 * NOTE - it is important to have at least a single '1' bit aftrer the mask
 * for example we want to intercept 0x90 = 0b10010000, and our mask would be 0x80=0b10000000
 *  So we could flip the second '1' bit.
 */
uint8_t address_mask = 0xE0;
uint8_t address_mask_len = 3;
uint8_t address_target = (0x90 & address_mask);
uint8_t data_to_write  = 0xbb;

/********************************************************************************************************
 *  Decl
 *********************************************************************************************************/
void print_i2c_buffer(BITS* buf_bits, size_t size);
void print_hex(unsigned int a);

void print_hex(unsigned int a) {
	char temp_buf[0x100];
	snprintf(temp_buf, sizeof(temp_buf), "0x%x ", a);
	puts(temp_buf);
}


void init_lcd(void) {
	// Init lcd log.
	STM322xG_LCD_Init();
	LCD_LOG_Init();
	LCD_LOG_SetHeader((uint8_t*) "I2C Sniffer");
}


/*
 * Packs the 9 bits to a byte and the ack.
 * returns true only when done
 */
bool read_byte(uint8_t &byte, bool &acked, BITS bit) {
	static uint8_t bit_count = 0;

	// First lets make sure it's indeed a 1/0 bit, and not a stop/start (validity...)
	if (bit != ZERO_BIT && bit != ONE_BIT) {
		puts("Saw a wierd bit. shouldn't happen\n");
	}
	// Recieve the first 8 bits
	if (bit_count < 8) {
		// If this is the first bit, zero the target byte.,
		if (bit_count == 0) {
			byte = 0;
		}

		// Pack up 8 bits
		uint8_t sa = ((uint8_t) (7-bit_count++));
		byte |= (uint8_t) (bit << sa);
		//byte = (uint8_t) (bit <<  4);
		return false; // Not done getting all the 9 bits
	} else {
		// The last bit is ack/nack (1-> nack, 0->ack)
		acked = !bit;

		// Prepare for the next byte.
		bit_count = 0;
		return true;
	}

}

bool examine_bits(BITS* bits_arr, size_t bits_size) {
	// The state of the i2c transaction.
	static I2C_STATE i2c_state = I2C_NOT_SYNCED;

	// The address in the current transaction
	static uint8_t address				= 0;
	static uint8_t address_bit_count 	= 0;

	// The read/write of the current transaction
	static bool is_write 			= 0;

	// Did a slave acked for the transaction?
	static bool is_slave_acked		= 0;

	// A temp buffer of the currently written byte.
	static uint8_t temp_byte = 0;

	static bool intercepted = false;
	//static std::vector<char> bytes_written;
	//static std::vector<char> bytes_read;

	bool hit_stop_bit = false;
	bool temp_ack;
	BITS last_bit = bits_arr[bits_size-1];


	/*
	 *  State machine of the i2c sniffer.
	 *  The
	 */
	switch (i2c_state) {
		case I2C_NOT_SYNCED:
			// In this case we are waiting for start bit. is the last bit captured a start bit?
			if (last_bit == START_BIT) {
				// clear vector on the first bit
				//bytes_written.clear();
				//bytes_read.clear();

				// Yes. lets move on and wait for the address to be transmitted.
				i2c_state 	= I2C_START_BIT;
				address_bit_count = 0;
			}

			break;

		case I2C_START_BIT:
			// Read the next bit in the address byte
			if (!read_byte(address, is_slave_acked, last_bit)) {
				address_bit_count++;
				// Gather enough bits to check address prefix
				if (address_bit_count == address_mask_len) {
					// Check if we have a match
					if 	((address & address_mask) == address_target) {
						// Yes. it matches, we need to flip the next ONE_BIT in the address currently being transmitted.
						i2c.flip_next_one_bit();
						// Also flag that the i2c should be the slave.
						intercepted = true;
						// TODO:We also need to remember to answer with ACK after all bytes finished.
					} else {
						// No. Let the request pass, don't intercept anything.
						intercepted = false;
					}
				}

				// If we intercepted the address, we need to ack after 8 bits of address.
				if ((address_bit_count == 8) && intercepted) {
					i2c.flip_next_one_bit();
				}
				break;
			}
			// We received 8 bits (+1 for ack), The last bit from address is read(1)/write(0) operation
			is_write = !(address & 0x1);
			// If we intercepted, and this is a read operation, the i2c module should act like a slave.
			if (!is_write) {
				i2c.writeByte(0xbb);
			}

			if (is_slave_acked) {
				// If the slave has acked -> move on to the next state.
				i2c_state = I2C_ADDRESS;
			} else {
				// If the slave didn't ack -> retry (TODO:?)
				// Retry? Go to previous stage? note that going back will flush both read/write vector.
				// Not sure...
				address_bit_count = 0;
				address = 0;
			}
			break;

		case I2C_ADDRESS:
			// Address was acked.
			// Is it stop or start bit?
			if (last_bit == START_BIT)  {
				// Its a start bit, wait for the address again...
				i2c_state = I2C_START_BIT;
			} else if (last_bit == STOP_BIT) {
				// Its a stop bit, wait for a new start bit...
				hit_stop_bit = true;
				i2c_state = I2C_NOT_SYNCED;
			} else {
				// Read the next bit and pack it into the byte
				if (!read_byte(temp_byte, temp_ack, last_bit)) {
					break;
				}
				// Was the byte acked?
				if (temp_ack) {
					if (is_write) {
						// If the master wrote this byte, push it to the written bytes.
						//bytes_written.push_back(temp_byte);
					} else {
						// If the slave wrote this byte, push it to the read bytes.
						//bytes_read.push_back(temp_byte);
					}
				} else {

					// No... the master should retransmit the internal address
					// retry to read another byte as the internal register address! TODO (?)
				}
			}
			break;




		default:
			puts("Shuold not get here - unknow i2c state\n");
			break;
	}

	return hit_stop_bit;
}

void do_input() {
	/*
	 * Start the main loop which samples the lines. and update the bits.
	 */
	size_t i = 0;
	BITS bit = ZERO_BIT;
	while (1) {
		// Sample lines
		if (i2c.Update()) {
			// A new bit captured? pop it from the buffer.
			if (!i2c.m_buffer.pop(bit)) {
				puts("Can't pop bit?\n");
			}
			// And push it to our bits buffer.
			unprocessed_bits[i++] = bit;

			// Parse the bit - this returns true at STOP bit
			if (examine_bits(unprocessed_bits, i)) {
				// If we hit stop bits, print, clear buffer and prepare for the next.
				print_i2c_buffer(unprocessed_bits, i);
				i = 0;
			}

		}

	}
}

void init_buttons() {
	STM_EVAL_PBInit(BUTTON_KEY, BUTTON_MODE_EXTI);
}




uint8_t pack_byte(BITS* bits) {
	uint8_t byte = 0;
	for (int8_t bit_index = 7, i = 0; bit_index>=0; ++i, bit_index--) {
		if (bits[i] != ZERO_BIT && bits[i]!= ONE_BIT) {
			puts("PROBLEM\n");
		}
		byte |= (uint8_t) (bits[i]<< bit_index);
	}
	return byte;
}

/*
 * Prints the bytes from I2C buffer
 */
void print_i2c_buffer(BITS* buf_bits, size_t size) {
	uint8_t temp_byte;
	bool 	acked;

	if (!size) { return; }

	for (size_t i = 0; i < size; ) {
		// In this case we are waiting for start bit. is the last bit captured a start bit?
		if (buf_bits[i] == START_BIT) {
			puts("[");
			i++;
		} else if  (buf_bits[i] == STOP_BIT) {
			puts("]");
			i++;
		} else {
			temp_byte = pack_byte(buf_bits+i);
			i += 8;
			acked = (buf_bits[i] == ZERO_BIT);
			i++;
			print_hex(temp_byte);
			if (acked) {
				puts("ACK ");
			} else {
				puts("NACK ");
			}
		}
	}
	puts("\n");
}
/**
 *  Button handler (KEY button is IRQ-15)
 *  This prints the buffer of the sniffed I2C msgs to the screen.
 */
#ifdef __cplusplus
extern "C" {
#endif
void EXTI15_10_IRQHandler(void) {
	EXTI_ClearFlag(EXTI_Line15);
	init_lcd();
}
#ifdef __cplusplus
}
#endif




int main(int argc, char* argv[]) {
	//Timer t;
	//t.start();

	// init the lcd display
	init_lcd();
	init_buttons();
	do_input();
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
