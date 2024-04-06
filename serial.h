#pragma once

#include "stdarg.h"
#include "stddef.h"

#include "stm32f103_md.h"

struct Ringbuffer {
    uint16_t head;  // writes happen here
    uint16_t tail;  // reads happen here
    uint8_t buf[512];  // size must be power of two
};

inline uint16_t ringbuffer_avail(struct Ringbuffer *rb) {
    return rb->head - rb->tail;
}  // 0..size -1
inline uint16_t ringbuffer_free(struct Ringbuffer *rb) {
    return sizeof(rb->buf) - (rb->head - rb->tail) - 1;
}  // size-1 .. 0
inline int ringbuffer_empty(struct Ringbuffer *rb) {
    return rb->head == rb->tail;
}

// _full tests for free < 2 instead of == 0 so that there's room for the final '\n' or 0x7e
inline int ringbuffer_full(struct Ringbuffer *rb) {
    return ringbuffer_free(rb) < 2;
}

// serial_init() initializes USART{1,2,3}.
//
// The bytes will transmitted be as 8 data bits, no Parity bit, 1 stop bit (8N1) at the specified baud rate.
//
// Before calling serial_init, make sure to set up the GPIO pins: TX to AF_PP/10MHz. RX to IN FLOATING or Pull-up.
// and to enable the USART in the RCC register:	RCC->APBxENR |= RCC_APBxENR_USARTyEN;
// the txbuf will be used by serial_printf* below.
void serial_init(struct USART_Type *usart, int baud, struct Ringbuffer *txbuf);

// Call this from void USARTx_IRQ_Handler(void) {...}
// it will copy the ringbuffer to the TX and disable transmission when the buffer is empty.
void serial_irq_handler(struct USART_Type *usart, struct Ringbuffer *rb);

// serial_printf() interprets fmt as a format string for the variable parameters and copies a corresponding
// message to the usarts ringbuffer for transmission.  if the buffer is full, zaps the buffer and prints "!OVFL!".
int serial_printf(struct USART_Type *usart, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

// same as above, but only prints into the buffer without starting the USART
int serial_printf_buf(struct USART_Type *usart, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

inline void serial_wait(struct USART_Type *usart) {
    while (usart->CR1 & USART_CR1_TXEIE)
        /*nix*/;
}