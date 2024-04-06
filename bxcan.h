#pragma once

// There is only CAN1, so the bxcan argument everywhere is a bit superfluous

#include "stm32f103_md.h"
#include <stddef.h>

extern volatile struct bxcanStatus {
    uint32_t tx_count;
    uint32_t tx_err;
    uint32_t tx_alst;
    uint32_t esr;
    uint32_t lec_count[8];
    uint32_t rx_count[2];
    uint32_t rx_ovfl[2];
} can1_status;

// APB1 is 36MHz, 18quanta/bit -> 2MHz/(1+div)
enum CANBaudRate {
    CAN_1MBd = 1,
    CAN_500KBd = 3,
    CAN_250Kbd = 7,
    CAN_125Kbd = 15,
    CAN_83Kbd = 23,
};

// bxcan_init initializes a bxCAN controller to operate at the given baudrate.
void bxcan_init(/*struct CAN_Type* bxcan,*/ enum CANBaudRate bd);

// bxcan_tx schedules a message for transmission.
// the header is in 32-bit packed (id-a[11], id-b[18], EXTID, RTR, REQ) format as defined in the CANTIxR registers.
// the lower two bits are forced to 01.
//
// bxcan_tx returns used mailbox: 0,1 2 on success, -1 if no transmit mailbox is available, -2 on other error
int bxcan_tx(/*struct CAN_Type* bxcan,*/ uint32_t header, size_t len, const uint8_t payload[8]);

struct CanFilter {
    uint32_t header;  //  32-bit packed (id-a[11], id-b[18], EXTID, X X)
    uint32_t mask;
    uint8_t fifo;  // 0, 1: matches go to fifo0 or fifo1, 0xff: end of list
};

// set filters 0..13 to scale:32-bit mode:mask.  filters should be terminated with a sentinel with fifo=0xff.
// depending on the hardware there can be 14 or 28 filters, but the f103_md has 14.
void bxcan_set_filters(/*struct CAN_Type* bxcan,*/ const struct CanFilter *filters);

// bxcan_tx pops the first message available in rx fifo <mbox> (0 or 1), or if that is empty returns -1.
// the header is in 32-bit packed (id-a[11], id-b[18], EXTID, X, X) format as defined in the CANTIxR registers. the
// return value is -1 if no message is available, or the index of the filter that matched this message. bxcan_rx is
// meant to be called from USB_LP_CAN1_RX0_IRQ_Handler or CAN1_RX1_IRQ_Handler and clears the IRQ status.
int bxcan_rx(/*struct CAN_Type* bxcan,*/ int mbox, uint32_t *header, size_t *len, uint8_t payload[8]);

void canstats(struct USART_Type *usart);

// send the timestamp of the previous time we were done sending this timestamp.
void bxcan_tx_ts(/*struct CAN_Type* bxcan,*/ uint32_t header);