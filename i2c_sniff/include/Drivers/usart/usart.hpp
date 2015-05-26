#ifndef __USART_HPP__
#define __USART_HPP__

void init_usart(uint32_t baud_rate);
bool uart_send(uint8_t elem);
bool uart_recieve(uint8_t &elem);

void uart_puts(const char* s);

unsigned int tx_buffer_free_count(void);

extern "C" void USART3_IRQHandler(void);
#endif /* __USART_HPP__ */
