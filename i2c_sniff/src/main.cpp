#include <stdio.h>
#include <vector>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include "lcd_log.h"
#include "Drivers\port.h"
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
/* I2Cx Communication boards Interface */
#define I2Cx                          I2C1
#define I2Cx_CLK                      RCC_APB1Periph_I2C1
#define I2Cx_EV_IRQn                  I2C1_EV_IRQn
#define I2Cx_ER_IRQn                  I2C1_ER_IRQn
#define I2Cx_EV_IRQHANDLER            I2C1_EV_IRQHandler
#define I2Cx_ER_IRQHANDLER            I2C1_ER_IRQHandler

#define I2Cx_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2Cx_SDA_PIN                  GPIO_Pin_9
#define I2Cx_SDA_GPIO_PORT            GPIOB
#define I2Cx_SDA_SOURCE               GPIO_PinSource9
#define I2Cx_SDA_AF                   GPIO_AF_I2C1

#define I2Cx_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2Cx_SCL_PIN                  GPIO_Pin_6
#define I2Cx_SCL_GPIO_PORT            GPIOB
#define I2Cx_SCL_SOURCE               GPIO_PinSource6
#define I2Cx_SCL_AF                   GPIO_AF_I2C1

// Fast i2c
#define I2C_SPEED 340000
#define I2C_DUTYCYCLE I2C_DutyCycle_16_9
//#define I2C_SPEED 100000
//#define I2C_DUTYCYCLE  I2C_DutyCycle_2
//#define SLAVE_ADDRESS 0x80
#define SLAVE_ADDRESS (0x12)
/**************************************************
 * Enums & Defines
 ***************************************************/

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
static bool scl_rise = false;

// The prefix of the address to intercept - Always starts with a start bit
//BITS address_to_intercept[] = { ONE_BIT, ZERO_BIT, ZERO_BIT }; // for 0xd0
// For 0xd/0xc --> 0b0000110X
BITS address_to_intercept[] = { ZERO_BIT, ZERO_BIT, ZERO_BIT, ONE_BIT};
// How many bits already matched.
uint8_t match_index = 0;
// A flag to signal interception is in progress in the current transaction
bool is_intercepting = false;
// A flag to indicate that an address bit is being flipped.
bool is_flip = false;
bool wait_for_start_bit = false;
uint32_t interception_count = 0;

/********************************************************************************************************
 *  Decl
 *********************************************************************************************************/
static inline void set_interrupt(FunctionalState state, EXTITrigger_TypeDef trigger, uint32_t port, uint8_t pin);


void init_sda_scl_interrupts();
void init_buttons();
static void init_gpio_port_interrupts(uint8_t port, uint8_t pin);

/*********************************************************************************************************
 *  Impl
 **********************************************************************************************************/

void init_lcd(void) {
	// Init lcd log.
	STM322xG_LCD_Init();
	LCD_LOG_Init();
	LCD_LOG_SetHeader((uint8_t*) "BARS I2C Sniffer");
}

/*
 * Configures an interrupt for raising/falling edge on Port.Pin
 */
static void init_gpio_port_interrupts(uint8_t port, uint8_t pin) {
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint8_t PIN_TO_EXTIX_IRQ[] = {
			  EXTI0_IRQn, // 0
			  EXTI1_IRQn, // 1
			  EXTI2_IRQn, // 2
			  EXTI3_IRQn, // 3
			  EXTI4_IRQn, // 4
			  EXTI9_5_IRQn, // 5
			  EXTI9_5_IRQn,  //6
			  EXTI9_5_IRQn,  //7
			  EXTI9_5_IRQn,  //8
			  EXTI9_5_IRQn,  //9
			  EXTI15_10_IRQn, // 10
			  EXTI15_10_IRQn, // 11
			  EXTI15_10_IRQn, // 12
			  EXTI15_10_IRQn, // 13
			  EXTI15_10_IRQn, // 14
			  EXTI15_10_IRQn  // 15

	};
	/* Enable the GPIO clock for the port */
	RCC_AHB1PeriphClockCmd(PORT_RCC_MASKx(port), ENABLE);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure Pin.Port pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = PORT_PIN_MASK(pin);
	GPIO_Init(PORT_GPIOx(port), &GPIO_InitStructure);

	/* Connect EXTI to the port. Note that PA.i/PB.i... are connected to line_i */
	SYSCFG_EXTILineConfig(port, pin);


	/* Configure EXTI linei, to capture Rising/Falling. */
//	EXTI_InitStructure.EXTI_Line = 1<<pin ;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line2 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = PIN_TO_EXTIX_IRQ[pin];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}


static inline void set_interrupt(FunctionalState state, EXTITrigger_TypeDef trigger, uint32_t port, uint8_t pin) {
	EXTI_InitTypeDef EXTI_InitStructure;
	/* Configure EXTI linei, to capture Rising/Falling. */
	EXTI_InitStructure.EXTI_Line = 1<<pin ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = trigger;
	EXTI_InitStructure.EXTI_LineCmd = state;
	EXTI_Init(&EXTI_InitStructure);

}

/**
 * @brief  Enables the I2C Clock and configures the different GPIO ports.
 * @param  None
 * @retval None
 */
static void I2C_Config(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	I2C_DeInit(I2Cx);
    I2C_SoftwareResetCmd(I2Cx, ENABLE);
    I2C_SoftwareResetCmd(I2Cx, DISABLE);
	/* RCC Configuration */
	/*I2C Peripheral clock enable */
	RCC_APB1PeriphClockCmd(I2Cx_CLK, ENABLE);

	/*SDA GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2Cx_SDA_GPIO_CLK, ENABLE);

	/*SCL GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2Cx_SCL_GPIO_CLK, ENABLE);

	/* Reset I2Cx IP */
	RCC_APB1PeriphResetCmd(I2Cx_CLK, ENABLE);

	/* Release reset signal of I2Cx IP */
	RCC_APB1PeriphResetCmd(I2Cx_CLK, DISABLE);

	/* GPIO Configuration */
	/*Configure I2C SCL pin */
	GPIO_InitStructure.GPIO_Pin = I2Cx_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);

	/*Configure I2C SDA pin */
	GPIO_InitStructure.GPIO_Pin = I2Cx_SDA_PIN;
	GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);

	/* Connect PXx to I2C_SCL */
	GPIO_PinAFConfig(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_SOURCE, I2Cx_SCL_AF);

	/* Connect PXx to I2C_SDA */
	GPIO_PinAFConfig(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_SOURCE, I2Cx_SDA_AF);

	/* NVIC configuration */
	/* Configure the Priority Group to 1 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Configure the I2C event priority */
	NVIC_InitStructure.NVIC_IRQChannel = I2Cx_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure I2C error interrupt to have the higher priority */
	NVIC_InitStructure.NVIC_IRQChannel = I2Cx_ER_IRQn;
	NVIC_Init(&NVIC_InitStructure);



	/* Initialize I2C peripheral */
	/*!< I2C Init */

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
	I2C_InitStructure.I2C_OwnAddress1 = SLAVE_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx, &I2C_InitStructure);
	/* Enable Error Interrupt */
	I2C_ITConfig(I2Cx, (I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), ENABLE);
	/* I2C ENABLE */
	I2C_Cmd(I2Cx, ENABLE);
	//I2C_ITConfig(I2Cx, I2C_IT_EVT , ENABLE);

}



void init_sda_scl_interrupts() {
	init_gpio_port_interrupts(SDA_PORT_IN, SDA_PIN_IN);
	init_gpio_port_interrupts(SCL_PORT_IN, SCL_PIN_IN);
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

/**
 * @brief  This function handles I2Cx event interrupt request.
 * @param  None
 * @retval None
 */
void I2Cx_EV_IRQHANDLER(void) {
	__IO uint32_t Event = 0x00;
	/* Get Last I2C Event */
	Event = I2C_GetLastEvent(I2Cx);
//	uint8_t temp;
	switch (Event) {
	/* ****************************************************************************/
	/*                          Slave Transmitter Events                          */
	/*                                                                            */
	/* ****************************************************************************/

	/* Check on EV1 */
	case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
		I2C_SendData(I2Cx, 0xdd);
		I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
		break;
		/* Check on EV3 */
	case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
	case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
		I2C_SendData(I2Cx, 0xbb);
		break;

		/* ****************************************************************************/
		/*                              Slave Receiver Events                         */
		/*                                                                            */
		/* ****************************************************************************/

		/* check on EV1*/
	case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
//		Rx_Idx = 0x00;
		break;

		/* Check on EV2*/
	case I2C_EVENT_SLAVE_BYTE_RECEIVED:
	case (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_SR1_BTF):
//		temp = I2C_ReceiveData(I2Cx);
		break;

		/* Check on EV4 */
	case I2C_EVENT_SLAVE_STOP_DETECTED:
		I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF);
		I2C_Cmd(I2Cx, ENABLE);
		break;

	default:
		break;
	}
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
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);

		// Get value of sda/scl
		bool sda = sdaPortIn.read();
		bool scl = sclPortIn.read();

		if (!sda && scl) {
			// sda is now 0 -> SDA Falls.
			// If scl is high, and SDA made a transition to 0 -> START BIT

			// 1) disable sda interrupt.
			set_interrupt(DISABLE, EXTI_Trigger_Falling, SDA_PORT_IN, SDA_PIN_IN);
			// 2) start clock interrupt.
			set_interrupt(ENABLE, EXTI_Trigger_Rising_Falling, SCL_PORT_IN, SCL_PIN_IN);
			is_intercepting = false;

		}
	}

}


static void inline reset_sda_interrupt() {
	// Disable the sda interrupt
	set_interrupt(DISABLE, EXTI_Trigger_Rising, SDA_PORT_IN, SDA_PIN_IN);
	// And reset the SDA interrupt for the start bit.
	set_interrupt(ENABLE, EXTI_Trigger_Falling, SDA_PORT_IN, SDA_PIN_IN);
	// reset the state.
	match_index = 0;
	wait_for_start_bit = true;

}
/*
 * SCL Interrupt (PA2)
 */
void EXTI2_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
		/* Clear the EXTI line 2 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line2);

		// Get value of sda/scl
		bool sda = sdaPortIn.read();
		bool scl = sclPortIn.read();


		if (is_flip) {
			is_flip = false;
			//puts("Stop!\n");
			sdaPortOut.write(1); // Pulled up.
			// Flipped a bit, start scannining for start bit again.
			reset_sda_interrupt();
		}
		// Only do it on rise.
		if (scl) {
			// Check if we have a matching bit
			if (address_to_intercept[match_index] == (BITS) sda) {
				match_index++;
			} else {
				// Not a matching address... start scanning for start bit all over again.
				// Disable the sda interrupt
				reset_sda_interrupt();
			}

			// in case of a matching bit, check if we have matched the entire address prefix.
			if (match_index == sizeof(address_to_intercept)) {
				// LCD_BarLog("Matched!\n");
				// Prepare a 0 on the sda line for next bit
				sdaPortOut.write(0);

				// Set the intercption flag, and mark that we are flipping a bit.
				is_intercepting = true;
				is_flip = true;
				interception_count++;
			}
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

	init_sda_scl_interrupts();
	// We start only the SDA interrupt.
	set_interrupt(ENABLE, EXTI_Trigger_Falling, SDA_PORT_IN, SDA_PIN_IN);
	sdaPortOut.write(1);

	while (1) {

	}
	I2C_Config();
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
