#include <stdio.h>
#include <vector>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include "Timer.h"
#include "BlinkLed.h"
#include "lcd_log.h"
#include "Drivers\port.h"

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
	ZERO_BIT = 0,
	ONE_BIT = 1,
	START_BIT = 2,
	STOP_BIT = 3,
};
/********************************************************************************************************
 *  Globals
 *********************************************************************************************************/


InPort		sdaPortIn(SDA_PORT_IN,	SDA_PIN_IN,
				GPIO_Speed_50MHz,
				GPIO_OType_OD,
				GPIO_PuPd_NOPULL);

OutPort		sdaPortOut(SDA_PORT_OUT,	SDA_PIN_OUT,
				GPIO_Speed_50MHz,
				GPIO_OType_OD,
				GPIO_PuPd_UP);


InPort		sclPortIn(SCL_PORT_IN,	SCL_PIN_IN,
				GPIO_Speed_50MHz,
				GPIO_OType_OD,
				GPIO_PuPd_NOPULL);

// A buffer to hold the bits captured from the i2c sniffer for actual processing
#define UNPROCCESED_BITS_SIZE 0x1000
static size_t bits_index = 0;
static BITS unprocessed_bits[UNPROCCESED_BITS_SIZE];

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
BITS 		address_to_intercept[] 	= { START_BIT, ONE_BIT, ZERO_BIT, ZERO_BIT };
// How many bits already matched.
uint8_t 	match_index 			= 0;
// A flag to signal interception is in progress in the current transaction
bool 		is_intercepting 		= false;
// A flag to indicate that an address bit is being flipped.
bool 		is_flip					= false;

/********************************************************************************************************
 *  Decl
 *********************************************************************************************************/
void		print_i2c_buffer(BITS* buf_bits, size_t size);
void 		print_hex(unsigned int a);
void 		init_gpio_interrupts();
void 		init_buttons();
uint8_t 	pack_byte(BITS* bits);
void 		init_i2c();




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

void 		init_i2c() {

}
void init_gpio_interrupts() {
	// TODO: Refactor this into a function and call it on both ports.
	// Makes PA.2, and PC.0 FLOATING INPUTS, with interrupts to capture fall/rise.
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIOA/GPIOAC clock -> Already done in ports...*/
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure PA.2 pin as input floating -> Already done in ports...*/
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	/* Configure PC.0 pin as input floating -> Already done in ports...*/
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect EXTI Line0 to PA pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);
//	  /* Connect EXTI Line1 to PC pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);

	/* Configure EXTI Line2, to capture Rising/Falling. */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

//	  /* Configure EXTI Line0, to capture Rising/Falling. */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line2 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
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
void print_i2c_buffer(BITS* buf_bits, size_t size) {
	uint8_t temp_byte;
	bool acked;

	if (!size) {
		return;
	}

	for (size_t i = 0; i < size;) {
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
}
/**
 * Interrupts must be as extern!
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

//int count1 = 0;
//int count2 = 0;
//int count3 = 0;
//int count4 = 0;
int sda_rise_count = 0;
int sda_fall_count = 0;

int scl_rise_count = 0;
int scl_fall_count = 0;

int false_sda_rise_count = 0;
int false_sda_fall_count = 0;
/**
 * SDA Interrupt (PC0)
 */
void EXTI0_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);

		// Get value of sda/scl
		bool sda = sdaPortIn.read();
		bool scl = sclPortIn.read();

		if (sda) {
			sda_rise_count++;
		} else {
			sda_fall_count++;
		}

		// if sda is now 1 -> SDA rise.
		// If scl is high, and SDA made transition to 1 -> STOP BIT
		if (sda && scl) {
			// Push the stop bit to the bits array
			unprocessed_bits[bits_index] = STOP_BIT;
			bits_index = (bits_index + 1) % UNPROCCESED_BITS_SIZE;

			// Since this is the end of the transaction, reset the matching index.
			match_index = 0;
			// Again - this is the transaction end -> We are no longer intercepting.
			is_intercepting = false;

			// Don't confuse it with a data bit! scl was floating high not as a part of a bit pulse.
			scl_rise = false;

			// Print the last transaction
			print_i2c_buffer(unprocessed_bits, bits_index);
			// And reset buffer.
			bits_index = 0;
		} else if (!sda && scl) {
			// sda is now 0 -> SDA Falls.
			// If scl is high, and SDA made a transition to 0 -> START BIT
			assert(bits_index == 0);
			unprocessed_bits[bits_index] = START_BIT;
			bits_index = (bits_index + 1) % UNPROCCESED_BITS_SIZE;
			// If we were intercepting the last transaction - we no longer are.
			is_intercepting = false;
			// Address always start at a start bit
			match_index++;
			// Don't confuse it with a data bit!
			scl_rise = false;
		} else if (sda) {
			false_sda_rise_count++;
		} else {
			false_sda_fall_count++;
		}
	}
}

/*
 * SCL Interrupt (PA2)
 */
void EXTI2_IRQHandler(void) {
	static bool temp_bit = 0;
	if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
		/* Clear the EXTI line 2 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line2);
		// Get value of sda/scl
		bool sda = sdaPortIn.read();
		bool scl = sclPortIn.read();
		if (scl) {
			scl_rise_count++;
		} else {
			scl_fall_count++;
		}

		// if scl is now 1 -> scl rise -> Some one xmiting a bit
		// Save the bit value, and wait for scl fall to add it
		if (scl) {
			temp_bit = sda;
			scl_rise = true;
		} else {
			// if scl is now 0 -> scl fall.
			// when scl falls the second time, we stop the flipping.
			if (is_flip) {
				is_flip = false;
				sdaPortOut.write(1); // Pulled up.
			}
			// if we had a rise before add this bit.
			// if not its probably a start/stop bit, ignore it.
			if (scl_rise) {
				unprocessed_bits[bits_index] = (BITS)temp_bit;
				bits_index = (bits_index + 1) % UNPROCCESED_BITS_SIZE;


				// Check if we have a matching bit
				if (match_index && (address_to_intercept[match_index] == (BITS) temp_bit)) {
					match_index++;
				}
				// Check if we matched the entire prefix.
				if (match_index == sizeof(address_to_intercept)) {
					// Set the intercption flag, and mark that we are flipping a bit.
					is_intercepting = true;
					is_flip = true;
					// Stop matching until next start
					match_index = 0;
					// Prepare a 0 on the sda line for next bit
					sdaPortOut.write(0);

				}
			}
			scl_rise = false;
		}
	}
}
#ifdef __cplusplus
}
#endif

int main(int argc, char* argv[]) {
	sdaPortOut.write(1);
	init_lcd();
	init_buttons();
	init_i2c();
	init_gpio_interrupts();
	sdaPortOut.write(1);

	while (1) {

	}

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
