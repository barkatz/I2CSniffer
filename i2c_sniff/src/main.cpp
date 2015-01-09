#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "Timer.h"
#include "BlinkLed.h"
#include "lcd_log.h"
#include "Drivers\port.h"
#include "Drivers\i2c\i2c.h"


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
	I2CSniffer i2c(SDA_PORT, SDA_PIN, SCL_PORT, SCL_PIN);

	/*
	 * Start the main loop which samples the lines. and update the bits.
	 */
	while (1) {
		i2c.Update();
	}
}


int main(int argc, char* argv[]) {
	// init the lcd display
	init_lcd();

	do_input();
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
