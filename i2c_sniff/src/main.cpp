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
	XMIT_BYTE_DONE, 	// When xmitting the last bit, we need to wait for it to finish,
						// and then switch to capture SCL rise and not fall.

	WRITE_BYTE,			// Master writes 8 bits of data (We are receiving)
	WRITE_BYTE_ACK,		// We write ack (Actually the real slave does ^^)
	WRITE_BYTE_UNK,		// In this state we read the first bit of the next data, or stop bit.

	DISABLED			// Disabled...
};

enum I2C_BYTE_HIT {
	I2C_BYTE_MATCHED,
	I2C_BYTE_MISS_MATCHED,
	I2C_BYTE_NOT_SEEN
};

enum I2C_BYTE_OP {
	OP_MATCH,				// The byte should be matched.
	OP_OVERRIDE,			// The byte should be overwritten
	OP_DONT_CARE			// A joker byte - it we ignore it.
};

enum I2C_SEQUNCE_TYPE {
	TYPE_BYTE,				// The sequence is a BYTE type
	TYPE_START,				// The sequence is of START BIT type.
	TYPE_STOP,				// The sequence is of STOP BIT type.
	TYPE_NAK,				// NAK bit
	TYPE_ACK				// ACK bit
};

/*
 * The following define a single link in a the sequence we are matching.
 */
struct i2c_sequence_t;
typedef struct i2c_sequence_t {
	I2C_BYTE_OP 			op;				// match or overwrite?
	I2C_SEQUNCE_TYPE 		type;			// a normal byte or start/stop bit?

	uint32_t 				actual_value;	// The actual value from the i2c bus
	uint32_t	 			value;			// The value we expect to match: valid only incase of MATCHING bytes.
	I2C_BYTE_HIT 			hit;			// Did we hit this sequence in the list?

	i2c_sequence_t* 			next;
} i2c_sequnce_t;

/*
 * USART Commands
 */
typedef struct commands_t {
	const char *command;
	void (*func) (void);
} commands_t;


// Easier macro
#define READ_SDA() 						read_port(SDA_PORT_IN, SDA_PIN_IN)
#define READ_SCL() 						read_port(SCL_PORT_IN, SCL_PIN_IN)
#define WRITE_SDA(val)					write_port(SDA_PORT_OUT, SDA_PIN_OUT, val)

#define CLEAR_PR(pin) 					EXTI->PR = 1 << (pin)

#define CAPTURE_FALLING_EDGE(pin) 		EXTI->RTSR &= ~(1 << (pin)); \
										EXTI->FTSR |= (1 << (pin))

#define CAPTURE_RISING_EDGE(pin) 		EXTI->FTSR &= ~(1 << (pin)); \
										EXTI->RTSR |= (1 << (pin))

#define PUSH_PULL_LINE(port,pin)		PORT_GPIOx(port)->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t) pin)); \
										PORT_GPIOx(port)->OTYPER |= (uint16_t) (GPIO_OType_PP << ((uint16_t) pin))

#define OPEN_DRAIN_LINE(port,pin) 		PORT_GPIOx(port)->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t) pin)); \
										PORT_GPIOx(port)->OTYPER |= (uint16_t) (GPIO_OType_OD	<< ((uint16_t) pin))

/********************************************************************************************************
 *  Decl
 *********************************************************************************************************/
/*
 * Reads from a port
 */
static bool inline read_port(uint32_t port, uint8_t pin);

/*
 * Writes to a port
 */
static void inline write_port(uint32_t port, uint8_t pin, bool val);

/*
 * Inits buttons behavior.
 */
static void init_buttons();

/*
 * Inits LCD display.
 */
static void init_lcd(void);

/*
 * Enables/Disable the module.
 */
static void enable();
static void disable();

/*
 * Matches the current sequence link.
 * This advances curr accordingly, and return true if it matched.
 */
static bool match_i2c_byte();
static bool match_stop_bit();
static bool match_start_bit();

/*
 * i2c sequence list related
 */
static void update_display_sequence();
static void inline advance_i2c_byte();
static void free_list(i2c_sequence_t* list);
static void command_update_sequence(void);
/*
 * Enables or disables sniff_mode according to the command
 */
static void command_update_sniff_mode(void);
// Sends the current byte  via serial (including symbol index! to make sure we didn't lose any symbols)
static void inline send_sequence(I2C_SEQUNCE_TYPE type);
/*
 * USART Command parsing
 */
static bool read_uart_command();


/********************************************************************************************************
 *  Globals
 *********************************************************************************************************/
static unsigned int			baudrate 				= 115200;		// The USART baudrate

static volatile uint8_t 	bit_count 				= 0; 		// Bit count for receiving/xmiting a byte

volatile unsigned int 		commands_count 			= 0x0; 		// The number of waiting USART commands in the queue
static 	char 				*cmd = 0;								// Current command

// State of the state machine that flips.
FLIP_STATE state;

// Bytes written/read counters

// Global counters
uint32_t start_bit_count			= 0;
uint32_t stop_bit_count				= 0;
uint32_t address_nacked 			= 0;
uint32_t write_byte_nacks 			= 0;
uint32_t bread 						= 0; // bytes read
uint32_t bwrite 					= 0; // bytes written

// This flag is to update display with a sequence:
// It should be changed:
// 1) When a new sequence is received
// 2) When a new sequence link is hit (to display the +)
bool update_display_sequnce = false;

// The head/curr pointer of the sequence list
i2c_sequence_t* i2c_byte_list_head		= NULL;
i2c_sequence_t* i2c_byte_list_curr 		= NULL;


char delim[] = " \t";
// Is this module in sniff mode? In this mode all bytes (including stop/start bits) are being sent to serial
bool sniff_mode = false;
size_t symbol_count = 0;

// Global commands array from usart
commands_t commands[] ={
		{.command="start", 	.func = enable},
		{.command="stop", 	.func = disable},
		{.command="seq", 	.func = command_update_sequence},
		{.command="sniff", 	.func = command_update_sniff_mode},
};

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


static void update_display_sequence() {
	//LCD_LOG_ClearTextZone();
	LCD_LOG_DeInit(); // Bah this ugly -> without this the lines appear on the bottom...
	LCD_LOG_SetLine(0);


	// format it...
	for (i2c_sequence_t* b = i2c_byte_list_head; b; b=b->next) {
		// If we hit this sequence link - print +
		if (b->hit == I2C_BYTE_MATCHED) {
			LCD_BarLog(LCD_COLOR_WHITE, "+");
		// If we hit it, but we missmatched it, and its a byte, print the value we saw in parentheses
		} else if ((b->hit == I2C_BYTE_MISS_MATCHED) && (b->type == TYPE_BYTE)) {
			LCD_BarLog(LCD_COLOR_WHITE, "(0x%lx)", b->actual_value);
		}
		switch (b->type) {
			case TYPE_BYTE:
				if (b->op == OP_OVERRIDE) {
					LCD_BarLog(LCD_COLOR_WHITE, "*");
				}

				if (b->op == OP_DONT_CARE) {
					LCD_BarLog(LCD_COLOR_WHITE, "? ");
				} else {
					LCD_BarLog(LCD_COLOR_WHITE, "0x%lx ", b->value);
				}

				break;
			case TYPE_START:
				LCD_BarLog(LCD_COLOR_WHITE, "[ ");
				break;
			case TYPE_STOP:
				LCD_BarLog(LCD_COLOR_WHITE, "] ");
				break;
			case TYPE_ACK:
			case TYPE_NAK:
					LCD_BarLog(LCD_COLOR_WHITE, "list is invalid.\n");
					break;
			}
	}
	LCD_BarLog(LCD_COLOR_WHITE, "\n");
	update_display_sequnce = false;
}

static void enable() {
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

	LCD_LOG_SetHeader((uint8_t*) "BARS I2C Sniffer(-ON-)", LCD_COLOR_BLUE, LCD_COLOR_WHITE);\
	update_display_sequence();
}

static void disable() {
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
	update_display_sequence();
}


static void inline advance_i2c_byte() {
	// If this is the first time this byte was hit -> we need to update the gui
	if (i2c_byte_list_curr->hit != I2C_BYTE_MATCHED) {
		update_display_sequnce = true;
	} else {
		update_display_sequnce = false;
	}

	i2c_byte_list_curr->hit = I2C_BYTE_MATCHED;
	i2c_byte_list_curr = i2c_byte_list_curr->next;
	i2c_byte_list_curr->actual_value = 0;
}

static bool match_i2c_byte() {
	// If we don't care, simply advance...
	if (i2c_byte_list_curr->op == OP_DONT_CARE) {
		advance_i2c_byte();
		return true;
	}

	// If it matches, advance...
	if ((i2c_byte_list_curr->op == OP_MATCH) && (i2c_byte_list_curr->value == i2c_byte_list_curr->actual_value)) {
		advance_i2c_byte();
		return true;

	} else if ((i2c_byte_list_curr->op == OP_MATCH) && (i2c_byte_list_curr->value != i2c_byte_list_curr->actual_value)) {
		i2c_byte_list_curr->hit = I2C_BYTE_MISS_MATCHED;
		update_display_sequnce = true;
	}
	// If it is overwritten, advance...
	if (i2c_byte_list_curr->op == OP_OVERRIDE) {
		advance_i2c_byte();
		return true;
	}

	// No match? back to square 1.
	i2c_byte_list_curr = i2c_byte_list_head;
	i2c_byte_list_curr->actual_value = 0;
	return false;
}

static bool match_start_bit() {
	if (i2c_byte_list_curr->op == OP_DONT_CARE) {
		advance_i2c_byte();
		return true;
	}

	if (i2c_byte_list_curr->op == OP_MATCH && i2c_byte_list_curr->type == TYPE_START) {
		advance_i2c_byte();
		return true;
	}


	i2c_byte_list_curr = i2c_byte_list_head;
	return false;
}


static bool match_stop_bit() {
	if (i2c_byte_list_curr->op == OP_DONT_CARE) {
		advance_i2c_byte();
		return true;
	}

	if (i2c_byte_list_curr->op == OP_MATCH && i2c_byte_list_curr->type == TYPE_STOP) {
		advance_i2c_byte();
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


		// Make sure we are matching start bit and NOT sniffing
		if (!sniff_mode && !match_start_bit()) {
			return;
		}

		// If we are sniffing, we need to send this byte via serial.
		if (sniff_mode) {
			send_sequence(TYPE_START);
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

static void inline scan_start_bit() {
	// Zero bit count
	bit_count = 0;
	// 1) disable SCL interrupt
	mask_interrupt(SCL_PIN_IN);
	// 2) enable SDA interrupt (searching for start bit)
	CAPTURE_FALLING_EDGE(SDA_PIN_IN);
	unmask_interrupt(SDA_PIN_IN);
}

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
			// If we are sniffing, we need to send this byte via serial.
			if (sniff_mode) {
				send_sequence(TYPE_BYTE);
			}
		}
		break; /* MATCH_ADDRESS */

	case ACK_ADDRESS_BIT:
		// Is it read or write address?
		is_read = i2c_byte_list_curr->actual_value & 1;

		// Check if the current actual_value matches the value expected... and that we are NOT sniffing
		if (!sniff_mode && !match_i2c_byte()){
			// Nope, start again.
			scan_start_bit();
			return;
		}

		// Check for NAK/ACK
		if (sda) {
			// NAK
			//-> Address doesn't exist. search for start bit. If we are sniffing send the nak
			address_nacked++;
			if (sniff_mode) {
				send_sequence(TYPE_NAK);
			}
			scan_start_bit();
		} else {
			// ACK -> Address found, slave answered.
			if (sniff_mode) {
				send_sequence(TYPE_ACK);
			}
			// If the next state is a OVERWRITE STATE we need to capture SCL fall. (to prepare the next bit).
			// In that case, the interrupt will be triggered when ACK SCL has fallen.
			// Note here - we can't change SDA to push/pull yet, since SCL is HIGH.
			if (!sniff_mode && (i2c_byte_list_curr->op == OP_OVERRIDE)) {
				CAPTURE_FALLING_EDGE(SCL_PIN_IN);
			}

			// Change state.
			state = is_read? READ_BYTE : WRITE_BYTE;
			bit_count = 0;
		}
		break;

	case READ_BYTE:
		// If we are overriding the byte (answering instead of the slave).
		if (!sniff_mode && (i2c_byte_list_curr->op == OP_OVERRIDE)){
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
			if (!sniff_mode && (i2c_byte_list_curr->op == OP_OVERRIDE)){
				// We are now xmitting the last bit, we'll need to change scl capture direction and realese SDA as soon
				// as we are done
				state = XMIT_BYTE_DONE;
			} else {
				// We are done with this byte -> Wait for the ack
				state = READ_BYTE_ACK;
			}
		}

		if (sniff_mode) {
			send_sequence(TYPE_BYTE);
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
		if (!sniff_mode && !match_i2c_byte()) {
			scan_start_bit();
			return;
		}

		if (sda) {
			// NACK -> Last byte the master wants to read. We expect stop bit.
			if (sniff_mode) {
				send_sequence(TYPE_NAK);
			}
			match_stop_bit();
			stop_bit_count++;
			scan_start_bit();
		} else {
			// Ack!
			if (sniff_mode) {
				send_sequence(TYPE_ACK);
			}
			// ACK -> Master wants to read more bytes
			bit_count = 0;
			// No actual value yet for the next byte.
			i2c_byte_list_curr->actual_value = 0;

			// If the next state is a OVERWRITE STATE we need to capture SCL fall. (to prepare the next bit).
			// In that case, the interrupt will be triggered when ACK SCL has fallen.
			if (!sniff_mode && (i2c_byte_list_curr->op == OP_OVERRIDE)) {
				CAPTURE_FALLING_EDGE(SCL_PIN_IN);
			}
			state = READ_BYTE;
		}
		break;

	case WRITE_BYTE:
		// If we are overriding the byte (replacing the master request...).
		if (!sniff_mode && (i2c_byte_list_curr->op == OP_OVERRIDE)) {
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
			if (!sniff_mode && (i2c_byte_list_curr->op == OP_OVERRIDE)) {
				// We are now xmitting the last bit, we'll need to change scl capture direction and realese SDA as soon
				// as we are done
				state = XMIT_BYTE_DONE;
			} else {
				// We are done with this byte -> Wait for the ack
				state = WRITE_BYTE_ACK;
			}
			bwrite++;
			if (sniff_mode) {
				send_sequence(TYPE_BYTE);
			}
		}

		break;

	case WRITE_BYTE_ACK:
		// Advance to the next byte.
		if (!sniff_mode && !match_i2c_byte()) {
			scan_start_bit();
			return;
		}

		if (sda) {
			// slave could not process it. nothing we can do
			write_byte_nacks++;
			send_sequence(TYPE_NAK);
		} else {
			// This is tricky. The master can now do 2 things:
			// A) Send another byte
			// B) Send a stop
			// We now wait for clock to go up again. then we will check if its a data bit or stop bit
			// No actual value yet for the next byte.
			send_sequence(TYPE_ACK);
			bit_count = 0;
			i2c_byte_list_curr->actual_value = 0;
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
				send_sequence(TYPE_STOP);
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

/*
 * Sends a sequence symbol via serial including an index.
 */
static void inline send_sequence(I2C_SEQUNCE_TYPE type) {
	// Reset for next one... and update index
	i2c_byte_list_curr->actual_value = 0;
	symbol_count++;
}

/*
 * Reads a complete uart command into cmd.
 * Returns true/false if there was a command in queue.
 */

static bool read_uart_command(){
	uint8_t 	 b;
	uint8_t 	size_low;
	uint8_t 	size_high;
	uint32_t  	size;
	// Check if there is any rdy command in the fifo
	if (commands_count) {
		// Read trash bytes untill # (start of cmd)
		while (1) {
			while(!uart_recieve(b));
			if (b == '#') {
				break;
			}
		}
		// Read 2 bytes for size
		while(!uart_recieve(size_high));
		while(!uart_recieve(size_low));
		size = ((size_high << 8) & (0xFF00))  | size_low;
		if (size > 0x1000) {
			uart_puts("Command is to big punk(>0x1000)\n");
			return false;
		}

		// Free prev command (it has been handled in the prev loop)
		if (cmd) {
			free(cmd);
			cmd = 0;
		}
		// Allocate room for the new command.
		cmd = (char*)malloc(size+1);

		// Recv it byte by byte.
		for (size_t i=0; i<size;i++) {
			while (!uart_recieve(b));
			cmd[i] = b;
		}

		// Decrement.
		commands_count--;

		// Null terminate
		cmd[size] = '\0';
		//uart_puts("Command recieved.\n");
		return true;
	} else {
		// No commands
		return false;
	}
}

/*
 * Free's a sequence list of bytes to trace
 */
static void free_list(i2c_sequence_t* list) {
	i2c_sequence_t *cur, *next;
	cur = list;
	next = cur->next;

	while (cur) {
		next = cur->next;
		memset(cur, 0, sizeof(*cur));
		free(cur);
		cur = next;
	}
}

/*
 * Switch into sniff mode.
 * In this mode all the start/stop bits and bytes are sent to serial.
 */
void command_update_sniff_mode(void) {
	char *tok;
	// Advance to the first sequence
	tok = strtok(NULL, delim);

	if (!strcmp(tok, "on")) {
		symbol_count = 0;
		sniff_mode = true;
	} else {
		sniff_mode = false;
	}
}

/*
 * Updates the global sequence list with given sequence taken from the global command.
 */
static void command_update_sequence(void) {
	char *tok;

	// First we free the old list.
	free_list(i2c_byte_list_head);

	// Create a new head, and make the curr list item point at it.
	i2c_byte_list_head = (i2c_sequence_t*) malloc(sizeof(i2c_sequence_t));
	i2c_byte_list_curr = i2c_byte_list_head;
	i2c_byte_list_curr->hit = I2C_BYTE_NOT_SEEN;

	// Advance to the first sequence
	tok = strtok(NULL, delim);

	// Tokenize the command.
	// Remember that the command is of form: [ 0x12 *0x33 0x22... ] )
	while (tok) {
		i2c_byte_list_curr->hit = I2C_BYTE_NOT_SEEN;
		i2c_byte_list_curr->actual_value 	= 0;
		if (tok[0] == '[') {
			// If the token is '[' we need to match a START bit
			i2c_byte_list_curr->op 				= OP_MATCH;
			i2c_byte_list_curr->type 			= TYPE_START;
		} else if (tok[0] == ']'){
			// If the token is '[' we need to match a STOP bit
			i2c_byte_list_curr->op 				= OP_MATCH;
			i2c_byte_list_curr->type 			= TYPE_STOP;
		} else if (tok[0] == '*') {
			// If the token starts with '*' we need to overwrite the next byte
			i2c_byte_list_curr->op 				= OP_OVERRIDE;
			i2c_byte_list_curr->type 			= TYPE_BYTE;
			i2c_byte_list_curr->value 			= strtol(tok+1, NULL, 0); // skip the *
		} else if (tok[0] == '?') {
			// If the token starts with '?' we don't care about this byte
			i2c_byte_list_curr->op 				= OP_DONT_CARE;
			i2c_byte_list_curr->type 			= TYPE_BYTE;
			i2c_byte_list_curr->value 			= 0;
		} else {
			// If its none of the above we just match the number
			i2c_byte_list_curr->op 				= OP_MATCH;
			i2c_byte_list_curr->type 			= TYPE_BYTE;
			i2c_byte_list_curr->value 			= strtol(tok, NULL, 0);
		}

		// On to the next item in the sequence
		tok = strtok(NULL, delim);

		// Allocate a new room for the next token if needed.
		if (tok) {
			i2c_byte_list_curr->next 		= (i2c_sequence_t*) malloc(sizeof(i2c_sequence_t));
		} else {
			i2c_byte_list_curr->next 	= NULL;
		}
		i2c_byte_list_curr 	= i2c_byte_list_curr->next;
	}

	// Start matching from the curr ptr...
	i2c_byte_list_curr = i2c_byte_list_head;
	update_display_sequence();

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
static void handle_command() {
	char* recved_cmd = strtok(cmd, delim);
	// Call the right command handler according to the first token
	for (size_t i=0; i < sizeof(commands)/sizeof(commands[0]); i++) {
		if (!strcmp(commands[i].command, recved_cmd)) {
			commands[i].func();
			uart_puts("OK\n");
			return;
		}
	}
	uart_puts("Unkown command!\n");

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


	size_t size = 0;
	while (1) {
		/*
		 * Handle command.
		 */
		if (read_uart_command()) {
			//LCD_BarLog(LCD_LOG_DEFAULT_COLOR,"%s\n", cmd);
			handle_command();
		}

		/*
		 * Handle status updates.
		 */
		if (update_display_sequnce) {
			update_display_sequence();
		}
	}


}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
