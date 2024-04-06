/*
 */
#include "stm32f103_md.h"

#include "bxcan.h"
#include "clock.h"
#include "gpio2.h"
#include "serial.h"

/*
    STM32F103CB (LQFP48/LQFP48) Pin Assignments:

    Pin   Function     DIR   Electrical     Connected to
    ---   ---------    ---   -----------    ---------------------------------------

    PA8   TIM1 CH1     in    PullUp         IR155 PWM output
    PA9   USART1 TX    out   AF_PP 10MHz    Debug Console 115200 8N1
    PA10  USART1 RX    in    PullUp
    PA11  CAN1 RX      in    PullUp         CAN transceiver RX  1Mbit
    PA12  CAN1 TX      out   AF_PP 50MHz    CAN transceiver TX  1Mbit
    PA13  SWDIO        in/out               ST-Link programmer
    PA14  SWCLK        in/out               ST-Link programmer

    PC13  LED0         out   OUT_OD 2MHz    On-board yellow LED
*/

enum {
    PWM_IN_PIN = PA8,
    USART1_TX_PIN = PA9,
    USART1_RX_PIN = PA10,
    CAN1_RX_PIN = PA11,
    CAN1_TX_PIN = PA12,
    LED0_PIN = PC13,
};

/* clang-format off */
static struct gpio_config_t {
    enum GPIO_Pin  pins;
    enum GPIO_Conf mode;
} pin_cfgs[] = {
    {PAAll, Mode_IN}, // reset
//  {PBAll, Mode_IN}, // reset
    {PCAll, Mode_IN}, // reset
	{PWM_IN_PIN, Mode_IPU},
    {USART1_TX_PIN, Mode_AF_PP_50MHz},
    {USART1_RX_PIN, Mode_IPU},
    {CAN1_TX_PIN, Mode_AF_PP_50MHz},
    {CAN1_RX_PIN, Mode_IPU},
    {LED0_PIN, Mode_Out_OD_2MHz},
    {0, 0}, // sentinel
};
/* clang-format on */

static inline void led0_on(void) {
    digitalLo(LED0_PIN);
}
static inline void led0_off(void) {
    digitalHi(LED0_PIN);
}
static inline void led0_toggle(void) {
    digitalToggle(LED0_PIN);
}

static struct Ringbuffer usart1tx;

/* clang-format off */
enum { IRQ_PRIORITY_GROUPING = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
struct {
    enum IRQn_Type irq;
    uint8_t        group, sub;
} irqprios[] = {
    {SysTick_IRQn,       0, 0},
    {TIM1_UP_IRQn,       1, 0},
    {TIM1_CC_IRQn,       1, 1},
    {USART1_IRQn,        3, 0},
    {None_IRQn, 0xff, 0xff},
};
/* clang-format on */

static uint16_t period = 0;
static uint16_t dutyc = 0;
void TIM1_UP_IRQ_Handler(void) {
    TIM1.SR &= ~TIM_SR_UIF;
    period = 0;
    dutyc  = 0;
}

void TIM1_CC_IRQ_Handler(void) {
    uint16_t sr = TIM1.SR;
    if (sr & TIM_SR_CC2IF) {
        dutyc = TIM1.CCR2;
        TIM1.SR &= ~TIM_SR_CC2IF;
    }
    if (sr & TIM_SR_CC1IF) {
        period = TIM1.CCR1;
        TIM1.SR &= ~TIM_SR_CC1IF;
    }

    if (period && dutyc) {
        serial_printf(&USART1, "% 5d/% 5d\eK\n", 5*(int)dutyc, 4*(int)period);
        period = 0;
        dutyc  = 0;        
    }
}


int main(void) {
    uint8_t rf = (RCC.CSR >> 24) & 0xfc;
    RCC.CSR |= RCC_CSR_RMVF;  // Set RMVF bit to clear the reset flags

    SysTick_Config(1U << 24);  // tick at 72Mhz/2^24 = 4.2915 HZ

    NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);
    for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
        NVIC_SetPriority(
                irqprios[i].irq, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, irqprios[i].group, irqprios[i].sub));
    }

    RCC.APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
    RCC.APB1ENR |= RCC_APB1ENR_TIM3EN;

    delay(10);  // let all clocks and peripherals start up

    for (const struct gpio_config_t *p = pin_cfgs; p->pins; ++p) {
        gpioConfig(p->pins, p->mode);
    }

    gpioLock(PAAll);
    gpioLock(PCAll);

    led0_off();

    serial_init(&USART1, 115200, &usart1tx);

    serial_printf(&USART1, "SWREV:%08x\n", __REVISION__);
    serial_printf(&USART1, "CPUID:%08lx\n", SCB.CPUID);
    serial_printf(&USART1, "DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
    serial_printf(
            &USART1, "RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "",
            rf & 0x20 ? " IWDG" : "", rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
    serial_wait(&USART1);

    // CC1,2 channel is configured as input, IC1,2 both mapped on TI1,  fSAMPLING=fDTS/8, N=8
    TIM1.CR1    = 2 << 8;  // CKD = /4, 18MHz
    TIM1.DIER   = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
    TIM1.PSC    = (CLOCKSPEED_HZ / 250000) - 1;  // 250Khz or 4us.
    TIM1.ARR    = 0xffff;  // overflow every 262.144ms
    TIM1.SMCR   = (0b101 << 4) | 0b100;  //  TS = 101 (TI1 is trigger), SMS = 100 (trigger is reset)
    TIM1.CCMR1  = (9 << 12) | (9 << 4) | (2 << 8) | 1;
    TIM1.CCMR2  = 0;
    TIM1.CCER   = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;  // 1,2 enabled, 2 inverted
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    TIM1.CR1 |= TIM_CR1_CEN;

    // Initialize the independent watchdog
    while (IWDG.SR != 0)
        __NOP();

    IWDG.KR = 0x5555;  // enable watchdog config
    IWDG.PR = 0;  // prescaler /4 -> 10kHz
    IWDG.RLR = 0xFFF;  // count to 4096 -> 409.6ms timeout
    IWDG.KR = 0xCCCC;  // start watchdog countdown

    for (;;)
        __WFI();

    return 0;
}
