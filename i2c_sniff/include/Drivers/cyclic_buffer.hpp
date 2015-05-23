#ifndef __CYCLIC_BUFFER_HPP__
#define __CYCLIC_BUFFER_HPP__

#define UART_BUF_SIZE 0x100


template <uint32_t N>
class CyclicBuffer {
public:
		CyclicBuffer(void) : m_head(0), m_tail(0) {}

		/* Return the buffer size */
		uint32_t size() const {return N;}

		/* Return how many elements are in the buffer */
		uint32_t count() const {
			if (m_tail >= m_head) {
				return m_tail - m_head;
			} else {
				return N - (m_head - m_tail - 1);
			}
		}

		/* Return true if buffer is full */
		bool is_full() const {
			return (count() == N) ? true : false;
		}

		/* Returns true if buffer is empty */
		bool is_empty() const {
			return (count() == 0) ? true : false;
		}

		bool push(uint8_t elem) {
			if (is_full()) {
				return false;
			}
			m_buf[m_tail] = elem;
			m_tail = (m_tail + 1) % N;
			return true;
		}

		bool pop(uint8_t &elem) {
			if (is_empty()) {
				return false;
			}
			elem =	m_buf[m_head];
			m_head = (m_head + 1) % N;
			return true;
		}

private:
	uint32_t m_head;
	uint32_t m_tail;
	uint8_t  m_buf[N];
};

#endif /*  __CYCLIC_BUFFER_HPP__ */
