
#include "serial.h"

extern inline uint16_t ringbuffer_avail(struct Ringbuffer *rb);
extern inline uint16_t ringbuffer_free(struct Ringbuffer *rb);
extern inline int ringbuffer_empty(struct Ringbuffer *rb);
extern inline int ringbuffer_full(struct Ringbuffer *rb);

static inline void put_head(struct Ringbuffer *rb, uint8_t c) {
    rb->buf[rb->head++ % sizeof(rb->buf)] = c;
}

extern inline void serial_wait(struct USART_Type *usart);

static struct Ringbuffer *tx_buf[3] = {NULL, NULL, NULL};

static inline int usart_index(struct USART_Type *usart) {
    if (usart == &USART1)
        return 0;
    if (usart == &USART2)
        return 1;
    if (usart == &USART3)
        return 2;

    for (;;)
        __NOP();  // hang

    return 0;
}

void serial_init(struct USART_Type *usart, int baud, struct Ringbuffer *txbuf) {
    const int idx = usart_index(usart);

    tx_buf[idx] = txbuf;

    usart->CR1 = 0;
    usart->CR2 = 0;
    usart->CR3 = 0;

    // PCLK2 for USART1, PCLK1 for USART 2, 3
    // this depends on SetSystemclockTo72MHz setting pclk1 to sysclk/2 and pclk1 to sysclc/2
    uint32_t clk = 72000000;
    if (usart != &USART1) {
        clk /= 2;
    }
    usart->BRR = clk / baud;

    static const enum IRQn_Type irqn[3] = {USART1_IRQn, USART2_IRQn, USART3_IRQn};
    NVIC_EnableIRQ(irqn[idx]);

    usart->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

void serial_irq_handler(struct USART_Type *usart, struct Ringbuffer *rb) {
    if (!ringbuffer_empty(rb)) {
        if ((usart->SR & USART_SR_TXE) != 0) {
            usart->DR = rb->buf[rb->tail++ % sizeof(rb->buf)];
        }
    } else {
        usart->CR1 &= ~USART_CR1_TXEIE;
    }
    return;
}

#define STB_SPRINTF_STATIC
#define STB_SPRINTF_MIN 32
#define STB_SPRINTF_NOFLOAT
#define STB_SPRINTF_IMPLEMENTATION

#include "stb_sprintf.h"

static char *rb_putcb(char *buf, void *user, int len) {
    struct Ringbuffer *rb = (struct Ringbuffer *)user;
    if (ringbuffer_free(rb) < len) {
        rb->head = rb->tail;
        put_head(rb, '!');
        put_head(rb, 'O');
        put_head(rb, 'V');
        put_head(rb, 'F');
        put_head(rb, '!');
    }
    for (int i = 0; i < len; ++i) {
        put_head(rb, buf[i]);
    }
    return buf;
}

// TODO: factor out (messy b/c of varargs)
int serial_printf_buf(struct USART_Type *usart, const char *fmt, ...) {
    const int idx = usart_index(usart);
    struct Ringbuffer *tx = tx_buf[idx];
    stbsp_set_separators('\'', '.');

    va_list ap;
    va_start(ap, fmt);
    char b[STB_SPRINTF_MIN];
    int rv = stbsp_vsprintfcb(rb_putcb, tx, b, fmt, ap);
    va_end(ap);

    return rv;
}

int serial_printf(struct USART_Type *usart, const char *fmt, ...) {
    const int idx = usart_index(usart);
    struct Ringbuffer *tx = tx_buf[idx];
    stbsp_set_separators('\'', '.');

    va_list ap;
    va_start(ap, fmt);
    char b[STB_SPRINTF_MIN];
    int rv = stbsp_vsprintfcb(rb_putcb, tx, b, fmt, ap);
    va_end(ap);

    usart->CR1 |= USART_CR1_TXEIE;  // enable transmissions

    return rv;
}
