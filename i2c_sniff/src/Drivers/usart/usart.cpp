#include "stm322xg_eval.h"
#include "Drivers/usart/usart.hpp"
#include "Drivers/cyclic_buffer.hpp"

extern volatile unsigned int commands;

#define UART_RX_BUFFER_SIZE 0x100
#define UART_TX_BUFFER_SIZE 0x100
static CyclicBuffer<UART_RX_BUFFER_SIZE> rx_buffer;
static CyclicBuffer<UART_TX_BUFFER_SIZE> tx_buffer;

void init_usart(uint32_t baud_rate) {

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EVAL_COM1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Init UART unit */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = baud_rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	STM_EVAL_COMInit(COM1, &USART_InitStructure);


	// And start the RX interrupt. Tx interrupt will be activated only when we have something to send.
	USART_ITConfig(EVAL_COM1, USART_IT_RXNE, ENABLE);

}

bool uart_send(uint8_t elem) {
	bool turn_on_int = false;

	// If tx queue is empty -> should turn on tx interrupt.
	if (tx_buffer.is_empty()) {
		turn_on_int = true;
	}
//
//	// try to push the elem to the tx queue. will not work if its full
	bool pushed = tx_buffer.push(elem);
//
//	// Enable the interrupt if needed.
	if (turn_on_int) {
		USART_ITConfig(EVAL_COM1, USART_IT_TXE, ENABLE);
	}

	return pushed;
}

bool uart_recieve(uint8_t &elem) {
	// Nothing to recieve?
	if (rx_buffer.is_empty()) {
		return false;
	}

	// Pop the recieved element.
	return rx_buffer.pop(elem);
}

void uart_puts(const char* s) {
	while(*s) {
		while (!uart_send(*s++));
	}
}

/*
 * Handles the UART interrupt!
 */

extern "C" void USART3_IRQHandler(void) {
	uint8_t b;
	// Is it a RX interrupt?
	if (USART_GetITStatus(EVAL_COM1, USART_IT_RXNE) != RESET) {
		/* Read one byte from the receive data register */
		b = USART_ReceiveData(EVAL_COM1) & 0x7F;
		if (rx_buffer.push(b) == false) {
			// This is bad.
		}
		// If this is a command end, increment the rdy command count
		if (b == '\n') {
			commands++;
		}
	}
	// Is it a TX interrupt?
	if (USART_GetITStatus(EVAL_COM1, USART_IT_TXE) != RESET) {

		// Pop an element from the tx queue
		uint8_t elem = 0;
		if (tx_buffer.pop(elem) == false) {
			// This is bad
		}
		/* Write one byte to the transmit data register */
		USART_SendData(EVAL_COM1, elem);

		// If the tx queue is empty, turn TX interrupt off
		if (tx_buffer.is_empty()) {
			USART_ITConfig(EVAL_COM1, USART_IT_TXE, DISABLE);
		}
	}
}

