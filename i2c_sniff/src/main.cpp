#include <stdio.h>
#include <vector>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include "stm32f2xx.h"
#include "stm32f2xx_rtc.h"
#include "BlinkLed.h"
#include "lcd_log.h"
#include "Drivers\port.h"
#include "CyclicBuffer.h"

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

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  PINOUT!
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*-------------------------------------------------------
 * IMPORTANT NOTE
 * ------------------------------------------------------
 * SDA/SCL Ports were chosen with extreme care since some ports are pulling the line to 0.
 * Yes. Even when configured as NO PULL, the lines where held down low.
 * I wrote down all the ports i have tried just for documentation, maybe i can figure out later on what
 * happened on those ports..
 * SDA port is - YELLOW -
 * SCL port is - WHITE -
 * //>>> Port PA.2 (CN1/37) - <<<<< THIS WORKS
 * //	 Port PA.3 (CN2/3) 	-  Pull to 0...
 * // 	 Port PA.13 (CN3/8) -  SWDIO no good don't use it (JTAG)
 * //	 Port PF.0 (CN1/15) -  Screen related... Not good...
 * // 	 Port PF.7 (CN1/23) -  Pull to 0 :(
 * //>>> Port PC.0 (CN1/29) - <<<< THIS WORKS
 * //>>> Port PE.2 (CN1/2) -  <<<< THIS WORKS
 */
// PC.0 (CN1/29) - SDA (YELLOW) ---> IN
#define SDA_PORT_IN     2
#define SDA_PIN_IN		0

// PE.2 (CN1/2) - SDA (YELLOW) ---> OUT
#define SDA_PORT_OUT    4
#define SDA_PIN_OUT		2

// PA.2 (CN1/37) - SCL (WITE)  ----> IN
#define SCL_PORT_IN		0
#define SCL_PIN_IN		2

// I2C PINS
// I2C1_SCL --> PB.6 (CN4/13)
// I2C1_SDA --> PB.9 (CN4/8)
#define I2C1_SCL
/**************************************************
 * Enums & Defines
 ***************************************************/
enum BITS {
	ZERO_BIT = 0, ONE_BIT = 1, START_BIT = 2, STOP_BIT = 3,
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

struct lines_samples_t {
	bool sda;
	bool scl;
};
uint32_t bla[0x100] = {0};
uint32_t bla2 = 0;
// A buffer to hold the bits captured from the i2c sniffer for actual processing
#define UNPROCCESED_BITS_SIZE 0x1000
static CyclicBuffer<BITS> unprocessed_bits(UNPROCCESED_BITS_SIZE);

/* This boolean flag used to flag scl_rise event.
 * We sample SDA when SCL falls, and treat it as a data bit.
 * This logic must check if there was an scl rise event before (not if for example, there was a start/end bit).
 */
static bool scl_rise = false;
/*
 * The address mask of the slave we are targeting.
 * This mask will be 'anded' with the current address of the slave and check if it matches the target.
 * If they match we will flip the next '1' bit.
 * NOTE - it is important to have at least a single '1' bit after the mask
 * for example we want to intercept 0x90 = 0b10010000, and our mask would be 0x80=0b10000000
 *  So we could flip the second '1' bit.
 */
uint8_t address_mask = 0xE0;
uint8_t address_mask_len = 3;
uint8_t address_target = (0x90 & address_mask);

// The prefix of the address to intercept - Always starts with a start bit
BITS address_to_intercept[] = { START_BIT, ONE_BIT, ZERO_BIT, ZERO_BIT };
// How many bits already matched.
uint8_t match_index = 0;
// A flag to signal interception is in progress in the current transaction
bool is_intercepting = false;
// A flag to indicate that an address bit is being flipped.
bool is_flip = false;

/********************************************************************************************************
 *  Decl
 *********************************************************************************************************/
size_t print_i2c_buffer(BITS* buf_bits, size_t size);
void print_i2c_buffer(CyclicBuffer<BITS>& buf_bits);
void print_hex(unsigned int a);
void init_buttons();
uint8_t pack_byte(BITS* bits);
void init_i2c();

/*********************************************************************************************************
 *  Impl
 **********************************************************************************************************/
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

void init_i2c() {

}

void init_buttons() {
	// Init the button interrupt
	STM_EVAL_PBInit(BUTTON_KEY, BUTTON_MODE_EXTI);
}

uint8_t pack_byte(BITS* bits) {
	uint8_t byte = 0;
	for (int8_t bit_index = 7, i = 0; bit_index >= 0; ++i, bit_index--) {
		assert((bits[i] == ONE_BIT) || (bits[i] == ZERO_BIT));
		byte |= (uint8_t) (bits[i] << bit_index);
	}
	return byte;
}

/*
 * Prints the bytes from I2C buffer
 */

size_t print_i2c_buffer(BITS* buf_bits, size_t size) {
	uint8_t temp_byte;
	bool acked;
	size_t i = 0;
	if (!size) {
		return 0;
	}

	for (i = 0; i < size;) {
		// In this case we are waiting for start bit. is the last bit captured a start bit?
		if (buf_bits[i] == START_BIT) {
			puts("[");
			i++;
		} else if (buf_bits[i] == STOP_BIT) {
			puts("]");
			i++;
		} else {
			temp_byte = pack_byte(buf_bits + i);
			i += 8;
			assert((buf_bits[i] == ONE_BIT) || (buf_bits[i] == ZERO_BIT));
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
	return i;
}
/*
 * Prints the bytes from I2C buffer
 */
void print_i2c_buffer(CyclicBuffer<BITS>& buf) {
	BITS temp;
	size_t processed_bits = print_i2c_buffer(buf.get_buffer(), buf.get_size());
	assert(processed_bits < buf.get_size());
	for (size_t i = 0; i < processed_bits; i++) {
		buf.pop(temp);
	}
}

void process_lines() {
	lines_samples_t prev_sample = { (bool) sdaPortIn.read(),
			(bool) sclPortIn.read() };
	lines_samples_t curr_sample = { (bool) sdaPortIn.read(),
			(bool) sclPortIn.read() };
	BITS temp_bit = ZERO_BIT;
	uint32_t bitcount = 0;
	uint32_t max_time = 0;
	uint32_t state_time = 0;
	volatile uint32_t start_time, end_time;


	while (1) {
		start_time = (volatile int) RTC->TR;
		curr_sample.sda = (bool) sdaPortIn.read();
		curr_sample.scl = (bool) sclPortIn.read();
		if ((curr_sample.sda == prev_sample.sda)
				&& (curr_sample.scl == prev_sample.scl)) {
			//state_time++;
			bla[bla2]++;
			continue;
		}
		bla2++;
		if (bla2 > 30) {
			for (size_t i=0; i<bla2; i++) {
				print_hex(bla[i]);
			}
			puts("\n");
		}
		goto loop_end;


		if (state_time > 0) {
			unprocessed_bits.push((BITS) temp_bit);
		}
		if (temp_bit == START_BIT) {
			puts("!");
			print_hex(state_time);
			puts("!\n");
		}
		// State has changed
		state_time = 0;
		if (prev_sample.sda && !curr_sample.sda) {
			/*
			 * SDA fall
			 */
			if (prev_sample.scl && curr_sample.scl) {
				/*
				 *  SDA fall, and high scl --> START BIT
				 */
				temp_bit = START_BIT;

				// If we were intercepting the last transaction - we no longer are.
				is_intercepting = false;
				// Address always start at a start bit
				match_index++;
				//	Don't confuse it with a data bit!
				scl_rise = false;
			}
		} else if (!prev_sample.sda && curr_sample.sda) {
			/*
			 * SDA raise
			 */
			if (prev_sample.scl && curr_sample.scl) {
				/*
				 *  SDA raise, and high scl --> STOP BIT
				 */
				// Stop bit must come after we finished recieving/xmitting a byte.
				// Note that we might get to this case by mistake, when sda and scl are changing together.
				if (bitcount) {
					goto loop_end;
				}

				// Push the stop bit to the bits array
				temp_bit = STOP_BIT;
				// Since this is the end of the transaction, reset the matching index.
				match_index = 0;
				// Again - this is the transaction end -> We are no longer intercepting.
				is_intercepting = false;
				// Don't confuse it with a data bit! scl was floating high not as a part of a bit pulse.
				scl_rise = false;
				// Print the last transaction
				print_i2c_buffer(unprocessed_bits);
			}
		} else if (!prev_sample.scl && curr_sample.scl) {
			/*
			 * SCL raise
			 */
			// Sample the bit
			temp_bit = (BITS) curr_sample.sda;
			scl_rise = true;
		} else if (prev_sample.scl && !curr_sample.scl) {
			/*
			 * SCL fall
			 */
			// when scl falls the second time, we stop the flipping.
			//if (is_flip) {
			//	is_flip = false;
			//	sdaPortOut.write(1); // Pulled up.
			//}
			//// if we had a rise before add this bit.
			//// if not its probably a start/stop bit, ignore it.
			if (scl_rise) {
				// Push the bit currently transmitted.
				unprocessed_bits.push((BITS) temp_bit);
				// Count the bit count in byte (8 bits +1 for ack).
				// this is needed to avoid detecting false start/stop bits in the middle of a byte transfer
				// due to accuracy issues (the transmitter might be changing sda together with scl really quickly
				// And we might consider it a stop/start bit by mistake)
				bitcount = (bitcount + 1) % 9;
				// Check if we have a matching bit
				//if (match_index && (address_to_intercept[match_index] == (BITS) temp_bit)) {
				//	match_index++;
				//}
				// Check if we matched the entire prefix.
				//if (match_index == sizeof(address_to_intercept)) {
				//	// Set the intercption flag, and mark that we are flipping a bit.
				//	is_intercepting = true;
				//	is_flip = true;
				//	// Stop matching until next start
				//	match_index = 0;
				//	// Prepare a 0 on the sda line for next bit
				//	sdaPortOut.write(0);
				//
				//}
				//}
			}
			scl_rise = false;
		} else {
			// This could happen?
			assert(0);
		}
		loop_end:
		end_time = (volatile int) RTC->TR;
		prev_sample = curr_sample;
		uint32_t diff = (end_time - start_time) % 0xFFFFFFFF;
		if (diff > max_time) {
			max_time = diff;
		}
		// Each tick is 1MHZ, 3 ticks is already bad.
		assert(diff < 3);

	}
}

ErrorStatus init_rtc() {
	RTC_InitTypeDef RTC_InitStruct;

	/* Enable the PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to RTC */
	PWR_BackupAccessCmd(ENABLE);

	/* LSI used as RTC source clock */
	/* The RTC Clock may varies due to LSI frequency dispersion. */
	/* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);

	/* Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {
	}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();
	RTC_StructInit(&RTC_InitStruct);
	RTC_InitStruct.RTC_SynchPrediv = 0;
	RTC_InitStruct.RTC_AsynchPrediv = 0;
	return RTC_Init(&RTC_InitStruct);
}

int main(int argc, char* argv[]) {
	sdaPortOut.write(1);
	init_lcd();
	init_buttons();
	init_i2c();
	if (init_rtc() != SUCCESS) {
		LCD_BarLog("Failed to init RTC!\n");
	}
	sdaPortOut.write(1);
	process_lines();

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
