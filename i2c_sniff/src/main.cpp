#include <stdio.h>
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

/*
 *  Global i2c sniffer.
 */
I2CSniffer i2c(SDA_PORT, SDA_PIN, SCL_PORT, SCL_PIN, 0x1000);

void print_hex(unsigned int a){
	char temp_buf[0x100];
	snprintf(temp_buf, sizeof(temp_buf), "0x%x ", a);
	puts(temp_buf);
}


void init_lcd(void) {
	// Init lcd log.
	STM322xG_LCD_Init();
	LCD_LOG_Init();
	LCD_LOG_SetHeader((uint8_t*) "I2C Sniffer");
	LCD_UsrLog("Start!\n\n");
}



void do_input() {

	/*
	 * Init sda and scl ports as inputs.
	 * Those has been chosen with carfull care since some ports are pulling the line to 0.
	 * Yes. Even when configured asd NO PULL, the lines where held down low.
	 * I wrote down all the ports i have tried just for documentation, maybe i can figure out later on what
	 * happend on those ports...
	 * // Port PA.2 (CN1/37) - SCL (BLUE) 							<---- THIS WORKS
	 * // Port PA.3 (CN2/3) - SDA (YELLOW) Pull to 0...
	 * // Port PA.13 (CN3/8) - SDA (YELLOW) --> SWDIO no good don't use it (JTAG)
	 * // Port PF.0 (CN1/15) - SDA (a-0,b-1,c-2,d-3,e-4,f-5) --> Screen related... Not good...
	 * // Port PF.7 (CN1/23) - SDA (a-0,b-1,c-2,d-3,e-4,f-5) --> Pull to 0 :(
	 * // Port PC.0 (CN1/29) - SDA (a-0,b-1,c-2,d-3,e-4,f-5) --> < ----- THIS WORKS
	 */

	/*
	 * Start the main loop which samples the lines. and update the bits.
	 */
	while (1) {
		i2c.Update();
	}
}

void init_buttons() {
	STM_EVAL_PBInit(BUTTON_KEY, BUTTON_MODE_EXTI);
}




uint8_t pack_byte(BITS* bits) {
	uint8_t byte = 0;
	for (int8_t bit_index = 7, i = 0; bit_index>=0; ++i, bit_index--) {
		if (bits[i] != ZERO_BIT && bits[i]!= ONE_BIT) {
			puts("PROBLEM");
		}
		byte |= (uint8_t) (bits[i]<< bit_index);
	}
	return byte;
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

	// Get the bits from the i2c module
	size_t size = i2c.m_buffer.get_size();

	// If there are none, bail
	if (!size) {
		return;
	}

	/*
	 * Extract the bits from the buffer;
	 */
	BITS bits[size];
	size_t i;
	BITS cur_bit = ZERO_BIT;

	for (i=0; i<size; ++i) {
		i2c.m_buffer.pop(cur_bit);
		bits[i] = cur_bit;
		switch (bits[i]){
			case ZERO_BIT:
				puts("0-");
				break;
			case ONE_BIT:
				puts("1-");
				break;
			case START_BIT:
				puts("[");
				break;
			case STOP_BIT:
				puts("]");
				break;
			default:
				puts("*");
				break;
		}
	}
	puts("\n");
	/*
	* Going over the bits, this time parsing the entire thing as bytes
	* First we search for the start bit
	*/
	for (i=0; i<size; ++i) {
		if (bits[i]== START_BIT) {


			break;
		} else {
			puts("?");
		}
	}

	while (size > 0) {
		if (bits[i] != START_BIT) {
			puts("OUT OF SYNC");
			return;
		} else {
			i++;
			size--;
			puts("START ");
		}
		/*
		 * Packet should look like this:
		 * | START BIT | <--- 7 BITS ADDRESS --> | R/W BIT | N/ACK | .... | STOP
		 * We validate that the packet size is ok.
		 */
		if (size < (1 + 7 + 1 + 1 + 1)) {
			puts("Packet size is too small... ");
			return;
		}

		/*
		 * Now we go and pack the 7 bits of address + 1 bit of R/W
		 * the MSB comes first
		 */
		uint8_t address = pack_byte(bits + i);
		print_hex(address);
		i += 8;
		size -= 8;

		uint8_t is_write = !(address&0x1);
		if (is_write) {
			puts("W ");
		} else {
			puts("R ");
		}

		/*
		 *Next bit should be NACK/ACK
		 */
		uint8_t is_nack = bits[i++];
		size--;
		if (is_nack) {
			puts("NACK ");
		} else {
			puts("ACK ");
		}

		/**
		 * If master is reading, keep on reading untill a NACK is seen.
		 */
		while (!is_nack && !is_write){
			/*
			 * Read a byte
			 */
			uint8_t byte = pack_byte(bits + i);
			print_hex(byte);
			i += 8;
			size -= 8;

			/*
			 * Next bit should be NACK/ACK
			 */
			is_nack = bits[i++];
			size--;
			if (is_nack) {
				puts("NACK ");
			} else {
				puts("ACK ");
			}
		}

		// We should have hit the STOP bit...
		if (bits[i] != STOP_BIT) {
			puts("No stop bit? ");
		} else {
			puts("STOP ");
		}
		i++;
		size--;
		puts("\n");
	}

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
