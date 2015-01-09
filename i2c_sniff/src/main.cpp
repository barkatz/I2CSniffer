//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "Timer.h"
#include "BlinkLed.h"
#include "lcd_log.h"
#include "Drivers\port.h"

// ----------------------------------------------------------------------------
//
// STM32F2 led blink sample (trace via ITM).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate POSIX retargetting, reroute the STDOUT and STDERR to the
// trace device and display messages on both of them.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
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

// Definitions visible only within this translation unit.
namespace {
// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
constexpr Timer::ticks_t BLINK_ON_TICKS = Timer::FREQUENCY_HZ * 2 / 3;
constexpr Timer::ticks_t BLINK_OFF_TICKS = Timer::FREQUENCY_HZ - BLINK_ON_TICKS;
}

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

void init_lcd(void) {
	// Init lcd log.
	STM322xG_LCD_Init();
	LCD_LOG_Init();
	LCD_LOG_SetHeader((uint8_t*) "I2C Sniffer");
	LCD_UsrLog("Start!\n\n");
}

//static char log_buffer[0x2000] = {'\x00'};
//static unsigned int log_buffer_offset=0;
//static unsigned int lcd_line_counter2 = 0;
//void log_buf(char* fmt, ...) {
//
//	va_list ap;
//	va_start(ap, fmt);
//	// Add the line number
//	log_buffer_offset += snprintf(log_buffer+log_buffer_offset,
//								sizeof(log_buffer) - log_buffer_offset,
//								"%d) ", lcd_line_counter2++);
//
//	// Add the content
//	log_buffer_offset += vsnprintf(log_buffer+log_buffer_offset, sizeof(log_buffer)-log_buffer_offset, fmt, ap);
//	va_end(ap);
//}
//
//void flush_log_buf() {
//	if (log_buffer_offset) {
//		LCD_BarLog(log_buffer);
//		log_buffer_offset = 0;
//	}
//}

void do_input() {

	/*
	 * Init sda and scl ports as inputs.
	 */
	// Port PA.2 (CN1/37) - SCL (BLUE)
	InPort scl(0, 2, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_NOPULL);
	// Port PA.3 (CN2/3) - SDA (YELLOW)
	// Port PA.13 (CN3/8) - SDA (YELLOW) --> SWDIO no good don't use it (JTAG)
	// Port PF.0 (CN1/15) - SDA (a-0,b-1,c-2,d-3,e-4,f-5) --> Screen related... Not good...
	// Port PF.7 (CN1/23) - SDA (a-0,b-1,c-2,d-3,e-4,f-5) --> Pull to 0 :(
	// Port PC.0 (CN1/29) - SDA (a-0,b-1,c-2,d-3,e-4,f-5) --> Pull to 0 :(
	InPort sda(2, 0, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_NOPULL);
	/*
	 * Start the main loop which samples the lines."scl %d sda %d\n\n", scl_val, sda_val
	 */
	// Read the current values of scl/sda
	Timer t;
	t.start();
	uint32_t scl_trans[0x100];
	uint32_t sda_trans[0x100];
	memset(scl_trans, 0, sizeof(scl_trans));
	memset(sda_trans, 0, sizeof(sda_trans));

	uint32_t sda_trans_count = 0, scl_trans_count = 0;

	uint32_t i = 0;
	uint8_t scl_old_val = scl.read();
	uint8_t sda_old_val = sda.read();
	LCD_BarLog("Start scl=%d\n", scl_old_val);
	LCD_BarLog("Start sda=%d\n", sda_old_val);

	while (1) {
		//t.sleep(1000);
		uint8_t scl_val = scl.read();
		uint8_t sda_val = sda.read();
		if (scl_val != scl_old_val) {
			//log_buf((char*)"scl=%d->%d\n", scl_old_val, scl_val);
			//LCD_BarLog("scl=%d->%d\n", scl_old_val, scl_val);
			scl_old_val = scl_val;
			scl_trans[scl_trans_count++] = i;
		}
		if (sda_val != sda_old_val) {
			//log_buf((char*)"sda=%d->%d\n", sda_old_val, sda_val);
			//LCD_BarLog("sda=%d->%d\n", sda_old_val, sda_val);
			sda_old_val = sda_val;
			sda_trans[sda_trans_count++] = i;
		}
		i++;
//		if ((i%50000) == 0) {
//			flush_log_buf();
//		}
	}
}


void do_output() {
	Timer t;
	t.start();

	/*
	 * Init sda and scl ports as inputs.
	 */
	// Port PA.2 (CN1/37) - SCL (BLUE)
	OutPort led1(6, 8, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_UP);
	// Port PA.3 (CN2/3) - SDA (YELLOW)
	// A - 0, B -1, C- 2, D-3 ,E-4, F- 5, G-6
	OutPort led2(6, 6, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_UP);

	OutPort scl(0, 2, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_NOPULL);
	OutPort sda(0, 3, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_NOPULL);
	/*
	 * Start the main loop which samples the lines."scl %d sda %d\n\n", scl_val, sda_val
	 */
	// Read the current values of scl/sda
	uint8_t i = 0;
	uint8_t leds = 0;
	uint32_t c = 0;
	while (1) {
		t.sleep(1);
		c++;
		i ^= 1;
		if ((c%300) == 0) {
			leds ^= 1;
			led1.write(leds);
			led2.write(leds);
		}
		sda.write(i);
		scl.write(i);

	}	// Infinite loop, never return.
}


int main(int argc, char* argv[]) {
	// init the lcd display
	init_lcd();

	//do_output();
	do_input();
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
