
#include "bxcan.h"
#include "clock.h"
#include "serial.h"

#include <string.h>

// There is only CAN1, so the bxcan argument everywhere is a bit superfluous

// mechanism for clock syncing; keep track of our clock when sending this packet was done last
static uint64_t last_tx_ts = 0;
static int ts_in_progres = -1;  // mbox currently transmitting ts: -1 is off, 012 is mbox

// transmit timestap with given header, recording internally the exact timestamp (in clockcycles) the transmit was
// finished. this timestamp is then transmitted as the payload in the next call.
void bxcan_tx_ts(/*struct CAN_Type* bxcan,*/ uint32_t header) {
    const uint8_t payload[8] = {last_tx_ts >> 56, last_tx_ts >> 48, last_tx_ts >> 40, last_tx_ts >> 32,
                                last_tx_ts >> 24, last_tx_ts >> 16, last_tx_ts >> 8,  last_tx_ts};
    ts_in_progres = bxcan_tx(/*bxcan, */ header, 8, payload);
}

volatile struct bxcanStatus can1_status;

static inline void update_can1_sts(int mbox, uint64_t now, uint8_t tsr) {
    ++can1_status.tx_count;
    if (tsr & CAN_TSR_TERR0)
        ++can1_status.tx_err;
    if (tsr & CAN_TSR_ALST0)
        ++can1_status.tx_alst;

    // if there was a tx_ts in progress, update it here.
    if ((ts_in_progres == mbox) && (tsr & CAN_TSR_TXOK0)) {
        last_tx_ts = now;
        ts_in_progres = -1;
    }
}

void USB_HP_CAN1_TX_IRQ_Handler(void) {
    uint64_t now = cycleCount();
    uint32_t tsr = CAN1.TSR;
    if (tsr & CAN_TSR_TME0) {
        update_can1_sts(0, now, tsr);
    }
    if (tsr & CAN_TSR_TME1) {
        update_can1_sts(1, now, tsr >> 8);
    }
    if (tsr & CAN_TSR_TME2) {
        update_can1_sts(2, now, tsr >> 16);
    }

    CAN1.TSR = CAN_TSR_RQCP2 | CAN_TSR_RQCP1 | CAN_TSR_RQCP0;
}

void CAN1_SCE_IRQ_Handler(void) {
    can1_status.esr = CAN1.ESR;
    can1_status.lec_count[(can1_status.esr >> 4) & 0x7]++;
    CAN1.MSR |= CAN_MSR_ERRI;  // write 1 to clear (rc_w1)
}

void bxcan_init(/*struct CAN_Type* bxcan,*/ enum CANBaudRate bd) {
    memset((void *)&can1_status, 0, sizeof can1_status);

    struct CAN_Type *bxcan = &CAN1;

    // enter init mode
    bxcan->MCR &= ~CAN_MCR_SLEEP;
    bxcan->MCR |= CAN_MCR_INRQ;
    while ((bxcan->MSR & CAN_MSR_INAK) == 0)
        __NOP();  // Wait

    bxcan->MCR &= ~(CAN_MCR_TXFP | CAN_MCR_RFLM | CAN_MCR_ABOM | CAN_MCR_NART | CAN_MCR_AWUM | CAN_MCR_TTCM);
    bxcan->MCR |= CAN_MCR_ABOM;  // automatic bus-off-management.

    bxcan->BTR = 0x014B0000 | ((uint32_t)bd & 0x1ff);  // -> 1+5+12 = 18 quanta per bit, at 36MHz

    bxcan->FMR |= CAN_FMR_FINIT;  // filters reset
    bxcan->FM1R = 0;  // all in mask mode
    bxcan->FS1R = CAN_FS1R_FSC;  // all in single 32-bit scale
    bxcan->FFA1R = 0;  // all assigned to fifo0
    bxcan->FA1R = 0;  // all deactivated
    bxcan->FMR &= ~CAN_FMR_FINIT;

    bxcan->IER = 0;
    bxcan->IER |= CAN_IER_TMEIE;
    bxcan->IER |= CAN_IER_FMPIE0 | CAN_IER_FOVIE0 | CAN_IER_FMPIE1 | CAN_IER_FOVIE1;
    bxcan->IER |= CAN_IER_EWGIE | CAN_IER_EPVIE | CAN_IER_BOFIE | CAN_IER_LECIE | CAN_IER_ERRIE;

    NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_RX1_IRQn);
    NVIC_EnableIRQ(CAN1_SCE_IRQn);

    bxcan->MCR &= ~CAN_MCR_INRQ;
    while ((bxcan->MSR & CAN_MSR_INAK) != 0)
        __NOP();  // Wait
}

int bxcan_tx(/*struct CAN_Type* bxcan,*/ uint32_t header, size_t len, const uint8_t payload[8]) {
    if (len > 8) {
        return -2;
    }
    struct CAN_Type *bxcan = &CAN1;
    int idx = -1;
    if (bxcan->TSR & CAN_TSR_TME0) {
        idx = 0;
    } else if (bxcan->TSR & CAN_TSR_TME1) {
        idx = 1;
    } else if (bxcan->TSR & CAN_TSR_TME2) {
        idx = 2;
    } else {
        return -1;
    }
    struct CAN_TxMailBox_Type *mbox = bxcan->sTxMailBox + idx;
    mbox->TIR = header & (CAN_TI0R_IDE | CAN_TI0R_EXID | CAN_TI0R_STID);
    mbox->TDTR = len & 0xf;  // max 8
    mbox->TDLR = ((uint32_t)payload[0]) | (((uint32_t)payload[1]) << 8) | (((uint32_t)payload[2]) << 16) |
            (((uint32_t)payload[3]) << 24);
    mbox->TDHR = ((uint32_t)payload[4]) | (((uint32_t)payload[5]) << 8) | (((uint32_t)payload[6]) << 16) |
            (((uint32_t)payload[7]) << 24);
    mbox->TIR |= CAN_TI0R_TXRQ;

    return idx;
}

void canstats(struct USART_Type *usart) {
    serial_printf(usart, "tx_count   %lu\n", can1_status.tx_count);
    serial_printf(usart, "tx_err     %lu\n", can1_status.tx_err);
    serial_printf(usart, "tx_alst    %lu\n", can1_status.tx_alst);
    serial_printf(usart, "esr        %lx\n", can1_status.esr);
    serial_printf(
            usart, "lec_cnt %lu %lu %lu %lu", can1_status.lec_count[0], can1_status.lec_count[1],
            can1_status.lec_count[2], can1_status.lec_count[3]);
    serial_printf(
            usart, "  %lu %lu %lu %lu\n", can1_status.lec_count[4], can1_status.lec_count[5], can1_status.lec_count[6],
            can1_status.lec_count[7]);
    serial_printf(usart, "rx_count  %lu %lu\n", can1_status.rx_count[0], can1_status.rx_count[1]);
    serial_printf(usart, "rx_ovfl   %lu %lu\n", can1_status.rx_ovfl[0], can1_status.rx_ovfl[1]);
}

void bxcan_set_filters(/*struct CAN_Type* bxcan,*/ const struct CanFilter *filters) {
    struct CAN_Type *bxcan = &CAN1;
    bxcan->FMR |= CAN_FMR_FINIT;
    bxcan->FM1R = 0;  // all in identifier-mask mode
    bxcan->FS1R = CAN_FS1R_FSC;  // all in Single 32-bit scale configuration
    bxcan->FA1R = 0;  // all off

    for (unsigned int i = 0; i < 14; ++i) {
        if (filters[i].fifo == 0xff)
            break;
        if (filters[i].fifo == 0) {
            bxcan->FFA1R &= ~(1 << i);
        } else {
            bxcan->FFA1R |= (1 << i);
        }
        bxcan->sFilterRegister[i].FR1 = filters[i].header;
        bxcan->sFilterRegister[i].FR2 = filters[i].mask;
        bxcan->FA1R |= (1 << i);
    }

    bxcan->FMR &= ~CAN_FMR_FINIT;
}

int bxcan_rx(/*struct CAN_Type* bxcan,*/ int mbox, uint32_t *header, size_t *len, uint8_t payload[8]) {
    struct CAN_Type *bxcan = &CAN1;
    ++can1_status.rx_count[mbox];

    volatile uint32_t *rfr = NULL;
    switch (mbox) {
    case 0:
        rfr = &bxcan->RF0R;
        break;
    case 1:
        rfr = &bxcan->RF1R;
        break;
    default:
        for (;;)
            __NOP();  // hang
    }
    if (*rfr & CAN_RF0R_FOVR0) {
        ++can1_status.rx_ovfl[mbox];
        *rfr |= CAN_RF0R_FOVR0;
    }
    if ((*rfr & CAN_RF0R_FMP0) == 0) {
        return -1;
    }

    *header = bxcan->sFIFOMailBox[mbox].RIR;
    *len = bxcan->sFIFOMailBox[mbox].RDTR & 0xf;
    uint8_t fmi = bxcan->sFIFOMailBox[mbox].RDTR >> 8;

    uint32_t wl = bxcan->sFIFOMailBox[mbox].RDLR;
    payload[0] = wl;
    payload[1] = wl >> 8;
    payload[2] = wl >> 16;
    payload[3] = wl >> 24;

    uint32_t wh = bxcan->sFIFOMailBox[mbox].RDHR;
    payload[4] = wh;
    payload[5] = wh >> 8;
    payload[6] = wh >> 16;
    payload[7] = wh >> 24;

    *rfr |= CAN_RF0R_RFOM0;
    return fmi;
}
