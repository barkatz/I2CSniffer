#include <stdio.h>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include "lcd_log.h"
//#include "Drivers\port.h"
#include "Drivers/timer/timer.hpp"
#include "Drivers/gpio/gpio.hpp"
#include "Drivers/usart/usart.hpp"
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
	MATCH_ADDRESS, 		// Match 7 address bits
	ACK_ADDRESS_BIT,	// Get Ack bit

	READ_BYTE,			// Master reads 8 bits of data (We are sending)
	READ_BYTE_ACK,		// Read 1 bit of ack/nack from master (We are receiving)
	XMIT_BYTE_DONE, 	// After the last bit is xmitted we need to change back to capture SCL clock rise

	WRITE_BYTE,			// Master writes 8 bits of data (We are receiving)
	WRITE_BYTE_ACK,		// We write ack (Actually the real slave does ^^)
	WRITE_BYTE_UNK,		// In this state we read the first bit of the next data, or stop bit.

	DISABLED			// Disabled...
};


enum I2C_BYTE_OP {
	OP_MATCH,
	OP_OVERRIDE,
	OP_DONT_CARE
};

enum I2C_BYTE_TYPE {
	TYPE_BYTE,
	TYPE_START,
	TYPE_STOP
};

struct i2c_byte;
typedef struct i2c_byte {
	I2C_BYTE_OP 	op;
	I2C_BYTE_TYPE 	type;

	uint32_t 		actual_value;
	uint32_t	 	value;
	bool 			hit;

	i2c_byte* next;
} i2c_byte;



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
static unsigned int			baudrate 				= 9600;

static volatile uint8_t 	bit_count 				= 0; 		// Bit count for receiving/xmiting a byte
uint8_t 					address_to_intercept 	= 0x1a;


volatile unsigned int 		commands 				= 0x0; 		// The number of waiting commands in the queue

// State of the state machine that flips.
FLIP_STATE state;

// Bytes written/read buffers.
uint32_t bread 						= 0;
uint32_t bwrite 					= 0;

// Global counters
uint32_t start_bit_count			= 0;
uint32_t stop_bit_count				= 0;
uint32_t address_nacked 			= 0;
uint32_t write_byte_nacks 			= 0;


i2c_byte* i2c_byte_list_head		= NULL;
i2c_byte* i2c_byte_list_curr 		= NULL;



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

/*
 * Enables/Disable interception mode.
 */
void enable();
void disable();

/*
 * Matches the current byte against the current i2c_byte.
 * This advances curr accordingly.
 */
bool match_i2c_byte();
bool match_stop_bit();
bool match_start_bit();

//void init_i2c_byte_list();


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
	LCD_LOG_SetHeader((uint8_t*) "BARS I2C Sniffer(-ON-)", LCD_COLOR_BLUE, LCD_COLOR_WHITE);
}

void init_buttons() {
	// Init the button interrupt
	STM_EVAL_PBInit(BUTTON_KEY, BUTTON_MODE_EXTI);
}



void enable() {
	// Start from matching address state.
	state = MATCH_ADDRESS;

	// Start matching the first byte in the sequence.
	i2c_byte_list_curr = i2c_byte_list_head;

	// Make sure interrupts are in the right direction (don't know which state we left).
	// -> Capture SDA FALL  -> Start bits
	// -> Capture SCL RAISE -> Normal bits
	CAPTURE_RISING_EDGE(SCL_PIN_IN);
	CAPTURE_FALLING_EDGE(SDA_PIN_IN);

	// Start looking for start bit.
	unmask_interrupt(SDA_PIN_IN);

	LCD_LOG_SetHeader((uint8_t*) "BARS I2C Sniffer(-ON-)", LCD_COLOR_BLUE, LCD_COLOR_WHITE);
}

void disable() {
	// Stop interrupts.
	mask_interrupt(SDA_PIN_IN);
	mask_interrupt(SCL_PIN_IN);

	// Change state to disabled
	state = DISABLED;

	// Restart matching from square one.
	i2c_byte_list_curr = i2c_byte_list_head;

	// Make sure SDA is not pulled up
	OPEN_DRAIN_LINE(SDA_PORT_OUT, SDA_PIN_OUT);
	WRITE_SDA(1);
	LCD_LOG_SetHeader((uint8_t*) "BARS I2C Sniffer(-OFF-)", LCD_COLOR_GREY, LCD_COLOR_RED);
}

bool match_i2c_byte() {
	// If we don't care, simply advance...
	if (i2c_byte_list_curr->op == OP_DONT_CARE) {
		i2c_byte_list_curr->hit = true;
		i2c_byte_list_curr = i2c_byte_list_curr->next;
		i2c_byte_list_curr->actual_value = 0;
		return true;
	}

	// If it matches, advance...
	if ((i2c_byte_list_curr->op == OP_MATCH) && (i2c_byte_list_curr->value == i2c_byte_list_curr->actual_value)) {
		i2c_byte_list_curr->hit = true;
		i2c_byte_list_curr = i2c_byte_list_curr->next;
		i2c_byte_list_curr->actual_value = 0;
		return true;
	}
	// If it is overwritten, advance...
	if (i2c_byte_list_curr->op == OP_OVERRIDE) {
		i2c_byte_list_curr->hit = true;
		i2c_byte_list_curr = i2c_byte_list_curr->next;
		i2c_byte_list_curr->actual_value = 0;
		return true;
	}

	// No match? back to square 1.
	i2c_byte_list_curr = i2c_byte_list_head;
	i2c_byte_list_curr->actual_value = 0;
	return false;
}

bool match_start_bit() {
	if (i2c_byte_list_curr->op == OP_DONT_CARE) {
		i2c_byte_list_curr->hit = true;
		i2c_byte_list_curr = i2c_byte_list_curr->next;
		i2c_byte_list_curr->actual_value = 0;
		return true;
	}

	if (i2c_byte_list_curr->op == OP_MATCH && i2c_byte_list_curr->type == TYPE_START) {
		i2c_byte_list_curr->hit = true;
		i2c_byte_list_curr = i2c_byte_list_curr->next;
		i2c_byte_list_curr->actual_value = 0;
		return true;
	}


	i2c_byte_list_curr = i2c_byte_list_head;
	return false;
}


bool match_stop_bit() {
	if (i2c_byte_list_curr->op == OP_DONT_CARE) {
		i2c_byte_list_curr->hit = true;
		i2c_byte_list_curr = i2c_byte_list_curr->next;
		i2c_byte_list_curr->actual_value = 0;
		return true;
	}

	if (i2c_byte_list_curr->op == OP_MATCH && i2c_byte_list_curr->type == TYPE_STOP) {
		i2c_byte_list_curr->hit = true;
		i2c_byte_list_curr = i2c_byte_list_curr->next;
		i2c_byte_list_curr->actual_value = 0;
		return true;
	}


	i2c_byte_list_curr = i2c_byte_list_head;
	return false;
}

/*
 * Test sequence to match:
 * [ 0x1A 0x3 ] [ 0x1B *0xaa *0xbb *0xcc *0xdd *0xee *0xff
 */

/**
 * Interrupts must be defined extern!
 */
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Button handler (KEY button is IRQ-15)
 * Enable or disable interception
 */
void EXTI15_10_IRQHandler(void) {

	if (state == DISABLED) {
		enable();
	} else {
		disable();
	}
	// Clear interrupt.
	EXTI_ClearFlag(EXTI_Line15);

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
		start_bit_count++;

		// Make sure we are matching start bit
		if (!match_start_bit()) {
			return;
		}
		// Move on.
		state = MATCH_ADDRESS;

		// 1) disable sda interrupt.
		mask_interrupt(SDA_PIN_IN);
		// 2) start clock interrupt - capture rising edge
		CAPTURE_RISING_EDGE(SCL_PIN_IN);
		unmask_interrupt(SCL_PIN_IN);
	}
	// Clear SDA Interrupt.
	CLEAR_PR(SDA_PIN_IN);

}

void inline scan_start_bit() {
	// Zero bit count
	bit_count = 0;
	// 1) disable SCL interrupt
	mask_interrupt(SCL_PIN_IN);
	// 2) enable SDA interrupt (searching for start bit)
	CAPTURE_FALLING_EDGE(SDA_PIN_IN);
	unmask_interrupt(SDA_PIN_IN);
}

bool bla_sda;
bool bla_scl;

bool bla_sda2;
bool bla_scl2;

/*
 * SCL Interrupt (PA2)
 */
void EXTI2_IRQHandler(void) {
	static bool 				is_read 			= 0;		// a flag to indiciate whether master is reading/writing
	bool 						stop_bit 			= false;	// Did we hit a stop bit after a write transaction.

	// Read lines
	bool scl = READ_SCL();
	bool sda = READ_SDA();

	switch (state) {
	case DISABLED:
		// Do nothing...
		break;

	// Are we reading the address
	case MATCH_ADDRESS:
		// Read the address to the current i2c_byte.
		i2c_byte_list_curr->actual_value <<= 1;
		i2c_byte_list_curr->actual_value |= sda;
		bit_count++;
		if (bit_count == 8) {
			state = ACK_ADDRESS_BIT;
		}
		break; /* MATCH_ADDRESS */

	case ACK_ADDRESS_BIT:
		// Is it read or write address?
		is_read = i2c_byte_list_curr->actual_value & 1;

		// Check if the current actual_value matches the value expected...
		if (!match_i2c_byte()){
			// Nope, start again.
			scan_start_bit();
			return;
		}

		// Check for NAK/ACK
		if (sda) {
			// NAK
			//-> Address doesn't exist. search for start bit.
			address_nacked++;
			scan_start_bit();
		} else {
			// ACK -> Address found, slave answered.

			// If the next state is a OVERWRITE STATE we need to capture SCL fall. (to prepare the next bit).
			// In that case, the interrupt will be triggered when ACK SCL has fallen.
			// Note here - we can't change SDA to push/pull yet, since SCL is HIGH.
			if (i2c_byte_list_curr->op == OP_OVERRIDE) {
				CAPTURE_FALLING_EDGE(SCL_PIN_IN);
			}

			// Change state.
			state = is_read? READ_BYTE : WRITE_BYTE;
			bit_count = 0;
		}
		break;

	case READ_BYTE:
		// If we are overriding the byte (answering instead of the slave).
		if (i2c_byte_list_curr->op == OP_OVERRIDE) {
			// set SDA OUT port to push/pull on the first bit
			// This can be done now since SCL is low.
			if (bit_count == 0) {
				PUSH_PULL_LINE(SDA_PORT_OUT, SDA_PIN_OUT);
			}
			// Write the current bit of data
			WRITE_SDA(i2c_byte_list_curr->value & (1<<(7-bit_count)));
		} else {
			// Not answering instead of the slave. just pack the bits
			i2c_byte_list_curr->actual_value <<= 1;
			i2c_byte_list_curr->actual_value |= sda;
		}

		bit_count++;

		// Finished the byte....
		if (bit_count == 8) {
			if (i2c_byte_list_curr->op == OP_OVERRIDE) {
				// We are now xmitting the last bit, we'll need to change scl capture direction and realese SDA as soon
				// as we are done
				state = XMIT_BYTE_DONE;
			} else {
				// We are done with this byte -> Wait for the ack
				state = READ_BYTE_ACK;
			}
		}
		// Update counters and wait for ack.
		bread++;
		break; /* READ_BYTE */

	case XMIT_BYTE_DONE:
		// Finished xmitting the last bit
		// Change back to capture rising edge of SCL, to capture the ACK bit
		CAPTURE_RISING_EDGE(SCL_PIN_IN);
		// Change back sda to open drain.
		OPEN_DRAIN_LINE(SDA_PORT_OUT, SDA_PIN_OUT);
		// Release it...
		WRITE_SDA(1);

		state = is_read? READ_BYTE_ACK : WRITE_BYTE_ACK;
		break;

	case READ_BYTE_ACK:
		// Advance to the next byte.
		if (!match_i2c_byte()) {
			scan_start_bit();
			return;
		}

		if (sda) {
			// NACK -> Last byte the master wants to read. We expect stop bit.
			match_stop_bit();
			stop_bit_count++;
			scan_start_bit();
		} else {
			// ACK -> Master wants to read more bytes
			bit_count = 0;
			// No actual value yet for the next byte.
			i2c_byte_list_curr->actual_value = 0;

			// If the next state is a OVERWRITE STATE we need to capture SCL fall. (to prepare the next bit).
			// In that case, the interrupt will be triggered when ACK SCL has fallen.
			if (i2c_byte_list_curr->op == OP_OVERRIDE) {
				CAPTURE_FALLING_EDGE(SCL_PIN_IN);
			}
			state = READ_BYTE;
		}
		break;

	case WRITE_BYTE:
		// If we are overriding the byte (replacing the master request...).
		if (i2c_byte_list_curr->op == OP_OVERRIDE) {
			// set SDA OUT port to push/pull on the first bit
			// This can be done now since SCL is low.
			if (bit_count == 0) {
				PUSH_PULL_LINE(SDA_PORT_OUT, SDA_PIN_OUT);
			}
			// Write the current bit of data
			WRITE_SDA(i2c_byte_list_curr->value & (1<<(7-bit_count)));
		} else {
			// Not answering instead of the slave. just pack the bits
			i2c_byte_list_curr->actual_value <<= 1;
			i2c_byte_list_curr->actual_value |= sda;
		}
		// One less bit to go...
		bit_count++;

		// Finished the byte....
		if (bit_count == 8) {
			if (i2c_byte_list_curr->op == OP_OVERRIDE) {
				// We are now xmitting the last bit, we'll need to change scl capture direction and realese SDA as soon
				// as we are done
				state = XMIT_BYTE_DONE;
			} else {
				// We are done with this byte -> Wait for the ack
				state = WRITE_BYTE_ACK;
			}
			bwrite++;
		}

		break;

	case WRITE_BYTE_ACK:
		// Advance to the next byte.
		if (!match_i2c_byte()) {
			scan_start_bit();
			return;
		}

		if (sda) {
			// slave could not process it. nothing we can do
			write_byte_nacks++;
		} else {
			// This is tricky. The master can now do 2 things:
			// A) Send another byte
			// B) Send a stop
			// We now wait for clock to go up again. then we will check if its a data bit or stop bit
			// No actual value yet for the next byte.
			bit_count = 0;
			i2c_byte_list_curr->actual_value = 0;
			state = WRITE_BYTE_UNK;
		}
		break;

	case WRITE_BYTE_UNK:
			// We captured SCL rise, this might be the next data bit or a stop bit.
			// The easiest way to check this is with a loop....
			mask_interrupt(SCL_PIN_IN);
			bla_scl = scl;
			bla_sda = sda;

			// check if sda is changing while scl is still high...
			do {
				if (sda != READ_SDA()) {
					// This means stop bit
					stop_bit = true;
				}
				scl = READ_SCL();
			} while (scl && !stop_bit);
			bla_scl2 = READ_SCL();
			bla_sda2 = READ_SDA();

			// if it was a stop bit, start scanning for start bit.
			if (stop_bit) {
				match_stop_bit();
				stop_bit_count++;
				scan_start_bit();
			} else {
				// if it wasn't a stop bit, it was bit 0 of the next byte
				bit_count = 1;
				i2c_byte_list_curr->actual_value = sda;
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

void print_stat(uint16_t line_num, uint16_t color, const char* fmt, uint32_t cur_counter, uint32_t prev_counter) {
	uint16_t c;
	// Clear line
	LCD_ClearLine((uint16_t) ((line_num + YWINDOW_MIN) * 24));
	// Set line
	LCD_LOG_SetLine(line_num);

	// If changed, change color.
	if (cur_counter == prev_counter) {
		c = LCD_COLOR_WHITE;
	} else {
		c = color;
	}
	LCD_BarLog(c, fmt, (unsigned int) cur_counter);
}

void update_lcd_stats() {
	uint16_t color;
	// Global counters
	static uint32_t prev_start_bit_count 			= 0;
	static uint32_t prev_stop_bit_count 			= 0;

	static uint32_t prev_address_nacked 			= 0;
	static uint32_t prev_write_byte_nacks 			= 0;

	// Not sure why we needc this - check this out later (if we drop this lines appear at bottom as well.)
	LCD_LOG_DeInit();
	print_stat((uint16_t) 0, LCD_COLOR_GREEN, 	"    <<<TARGET ADDRESS 0x%08x>>>\n", address_to_intercept, address_to_intercept);
	print_stat((uint16_t) 1, LCD_COLOR_GREEN, 	"Start bit count           = 0x%08x\n", start_bit_count, prev_start_bit_count);
	print_stat((uint16_t) 2, LCD_COLOR_RED, 	"Stop bit count            = 0x%08x\n", stop_bit_count, prev_stop_bit_count);
	print_stat((uint16_t) 3, LCD_COLOR_RED, 	"Nacked addresses          = 0x%08x\n", address_nacked, prev_address_nacked);
	print_stat((uint16_t) 4, LCD_COLOR_RED, 	"Write nacks               = 0x%08x\n", write_byte_nacks, prev_write_byte_nacks);

	// Update counters
	prev_start_bit_count		= start_bit_count;
	prev_stop_bit_count			= stop_bit_count;
	prev_write_byte_nacks		= write_byte_nacks;
	prev_address_nacked 	 	= address_nacked;
}


/*
 * Reads a complete uart command into cmd.
 * Returns true/false if there was a command in queue.
 */
size_t read_uart_command(uint8_t* cmd){
	size_t i = 0;
	uint8_t 	 b;
	// Check if there is any rdy command in the fifo
	if (commands) {
		// Read until the end of the command ('\n')
		while (uart_recieve(b)) {
			if (b == '\n') {
				break;
			}
			cmd[i++] = b;
		}
		// Decrement.
		commands--;

		// Null terminate
		cmd[i] = '\0';
		return i;

	} else {

		return 0;
	}
}

/*
 * Free's a sequence list of bytes to trace
 */
void free_list(i2c_byte* list) {
	i2c_byte *cur, *next;
	cur = list;
	next = cur->next;

	while (cur) {
		next = cur->next;
		free(cur);
		cur = next;
	}
}

/*
 * Handles a cmd.
 * The cmd is a byte sequence to match on the i2c bus:
 * An example would be:
 * [ 0x26 3] [0x27 *0x30 ? 0x30
 * [ ] 		--> Start/Stop bits
 * 0x26 	--> Byte to match
 * *0x30 	--> Byte to overwrite
 * ? 		--> Don't care byte
 */
void handle_command(char* cmd, size_t size) {
	char delim[] = " \t";
	char* tok = strtok(cmd, delim);

	// Is this merely a stop/start command
	if (!strcmp(tok, "stop")) {
		disable();
		return;
	} else if (!strcmp(tok, "start")) {
		enable();
		return;
	}

	// No, this is a sequence we should follow. Lets ditch the old sequence
	free_list(i2c_byte_list_head);

	// Create a new head, and make the curr list item point at it.
	i2c_byte_list_head = (i2c_byte*) malloc(sizeof(i2c_byte));
	i2c_byte_list_curr = i2c_byte_list_head;
	i2c_byte_list_curr->hit = false;

	// Tokenize the command.
	// Remember that the command is of form: [ 0x12 *0x33 0x22... ] )
	while (tok) {

		if (tok[0] == '[') {
			// If the token is '[' we need to match a START bit
			i2c_byte_list_curr->op 				= OP_MATCH;
			i2c_byte_list_curr->type 			= TYPE_START;
			i2c_byte_list_curr->actual_value 	= 0;


		} else if (tok[0] == ']'){
			// If the token is '[' we need to match a STOP bit
			i2c_byte_list_curr->op 				= OP_MATCH;
			i2c_byte_list_curr->type 			= TYPE_STOP;
			i2c_byte_list_curr->actual_value 	= 0;

		} else if (tok[0] == '*') {
			// If the token starts with '*' we need to overwrite the next byte
			i2c_byte_list_curr->op 				= OP_OVERRIDE;
			i2c_byte_list_curr->type 			= TYPE_BYTE;
			i2c_byte_list_curr->value 			= strtol(tok+1, NULL, 0); // skip the *
			i2c_byte_list_curr->actual_value 	= 0;

		} else if (tok[0] == '?') {
			// If the token starts with '?' we don't care about this byte
			i2c_byte_list_curr->op 				= OP_DONT_CARE;
			i2c_byte_list_curr->type 			= TYPE_BYTE;
			i2c_byte_list_curr->value 			= 0;
			i2c_byte_list_curr->actual_value 	= 0;

		} else {
			// If its none of the above we just match the number
			i2c_byte_list_curr->op 		= OP_MATCH;
			i2c_byte_list_curr->type 	= TYPE_BYTE;
			i2c_byte_list_curr->value 	= strtol(tok, NULL, 0);
			i2c_byte_list_curr->actual_value = 0;
		}

		// On to the next item in the sequence
		tok = strtok(NULL, delim);

		// Allocate a new room for the next token if needed.
		if (tok) {
			i2c_byte_list_curr->next 		= (i2c_byte*) malloc(sizeof(i2c_byte));
			i2c_byte_list_curr->next->hit 	= false;
		} else {
			i2c_byte_list_curr->next 	= NULL;
		}
		i2c_byte_list_curr 	= i2c_byte_list_curr->next;
	}

	// Start matching from the curr ptr...
	i2c_byte_list_curr = i2c_byte_list_head;
}

int main(int argc, char* argv[]) {
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
	init_usart(baudrate);

	// Start looking for start bit.
	// unmask_interrupt(SDA_PIN_IN);
	// Start disabled.
	disable();


	uint8_t cmd[0x100];
	size_t size = 0;
	while (1) {
		if ( (size = read_uart_command(cmd)) ) {
			LCD_BarLog(LCD_LOG_DEFAULT_COLOR,"%s\n", cmd);
			handle_command((char*) cmd, size);
		}
	}


}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
