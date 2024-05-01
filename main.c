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
    PA6   TIM3 CH1     out   AP_PP 2MHz     debug pwm output 10Hz 33% signal

    PA8   TIM1 CH1     in    PullUp         IR155 PWM output (5V tolerant)
    PA9   USART1 TX    out   AF_PP 10MHz    Debug Console 115200 8N1
    PA10  USART1 RX    in    PullUp
    PA11  CAN1 RX      in    PullUp         CAN transceiver RX  1Mbit
    PA12  CAN1 TX      out   AF_PP 50MHz    CAN transceiver TX  1Mbit
    PA13  SWDIO        in/out               ST-Link programmer
    PA14  SWCLK        in/out               ST-Link programmer

    PC13  LED0         out   OUT_OD 2MHz    On-board yellow LED
*/

enum {
    PWM_OUT_PIN = PA6,
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
	{PWM_OUT_PIN, Mode_AF_PP_2MHz},
	{PWM_IN_PIN, Mode_IPU},
    {USART1_TX_PIN, Mode_AF_PP_50MHz},
    {USART1_RX_PIN, Mode_IPU},
    {CAN1_TX_PIN, Mode_AF_PP_50MHz},
    {CAN1_RX_PIN, Mode_IPU},
    {LED0_PIN, Mode_Out_OD_2MHz},
    {0, 0}, // sentinel
};
/* clang-format on */

static inline void led0_on(void)     { digitalLo(LED0_PIN); }
static inline void led0_off(void)    { digitalHi(LED0_PIN); }
static inline void led0_toggle(void) { digitalToggle(LED0_PIN); }

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

static struct Ringbuffer usart1tx;
void USART1_IRQ_Handler(void) { serial_irq_handler(&USART1, &usart1tx); }

enum {
    HDR29_BASE        = 0x19690908,

    // flags to compose the message (bit [16...13])
    MSG_NORMALOP        = 0x1,  // payload present: uint16 resistance in kOhm (10/20Hz)
    MSG_UNDERVOLT       = 0x2,  // combined with normal op, not a fault (20Hz)
    MSG_SSM             = 0x4,  // speed start measurement mode (30Hz)
    MSG_FAULT           = 0x8,  // something is wrong:

    MSG_NO_SIGNAL       = MSG_FAULT,                  // 0x8 no pwm detected
    MSG_INSFAULT        = MSG_FAULT | MSG_NORMALOP,   // 0x9  10Hz with R < 100K
    MSG_GROUNDFLT       = MSG_FAULT | MSG_UNDERVOLT,  // 0xA  50Hz/50%
    MSG_INSFAULT_UV     = MSG_FAULT | MSG_NORMALOP | MSG_UNDERVOLT,   // 0xB  20Hz with R < 100K 
    MSG_INSFAULT_SSM    = MSG_FAULT | MSG_SSM,                 // 0xC   30Hz, >90%
    MSG_RESERVED        = MSG_FAULT | MSG_SSM | MSG_NORMALOP,  // 0xD not used
    MSG_DEVFLT          = MSG_FAULT | MSG_UNDERVOLT | MSG_SSM, // 0xE 40Hz/50% device fault 
    MSG_INVALID         = MSG_FAULT | 7,                       // 0xF outside of valid period/dutyc bounds

};

const char* label[] = { "<0>","NORMALOP","<2>","UNDERVOLT","SSM","<5>","<6>","<7>","NO_SIGNAL","INSFAULT","GROUNDFLT","INSFAULT_UV","INSFAULT_SSM","<D>","DEVFLT","INVALID",};

static uint32_t mkhdr(uint32_t msgid) {  
    uint32_t r = HDR29_BASE | (msgid & 0xf)<<12; 
    return (r << 3) | CAN_TI0R_IDE;
}

static struct { int period; int freq; } p2f[] = {
    {18000, 0},
    {22000, 50},
    {24000, 0},
    {26000, 40},
    {30000, 0},
    {35000, 30},
    {48000, 0},
    {52000, 20},
    {90000, 0},
    {110000, 10},
    {__INT_MAX__, 0},
};

static uint32_t lastmsg = 0;
static uint64_t last_us = 0;

int limit_msg(uint32_t header, size_t len, uint8_t payload[8]) {
    uint64_t now_us = cycleCount()/C_US;
    if ((lastmsg == header) && ((now_us - last_us) < 90000))
        return -1;
    lastmsg = header;
    last_us = now_us;

    return bxcan_tx(header, len, payload);
}

int sendmsg(int period_us, int dutyc_us) {

    serial_printf(&USART1, "%lld  % 5d/% 5d\e[K\n", cycleCount()/C_US, dutyc_us, period_us);

    uint8_t buf[] = {0,0,0,0,0,0,0,0};
    if (period_us == 0) {
        return limit_msg(mkhdr(MSG_NO_SIGNAL), 0, buf);
    }

    if ((dutyc_us*20 < period_us) || (period_us - dutyc_us)*20 < period_us) {
        // not a valid duty cycle
        return limit_msg(mkhdr(MSG_INVALID), 0, buf);
    }

    int freq = 0;
    for (int i = 0; ; ++i) {
        if (period_us < p2f[i].period) {
            freq = p2f[i].freq;
            break;
        }
    }

    uint32_t msgid = 0;
    switch (freq) {
    case 20:
        msgid = MSG_UNDERVOLT;
        // fallthrough
    case 10:
        msgid += MSG_NORMALOP;
        if (dutyc_us*20 < period_us)  // less than 5%
            break; 
        if ((period_us - dutyc_us)*20 < period_us)  // more than 95%
            break; 

        // dc = duty_us / period_us
        // rf = (.9 x 1200k) / (dc - .05)  - 1200k
        // rf = (.9 * 1200k) / (duty_us / period_us - 0.05) - 1200k
        // rf = (9000 * 1200k) / (10000*duty_us / period_us - 500) - 1200k
        int rf = ((9000 * 1200) / (((10000*dutyc_us) / period_us) - 500)) - 1200;
        buf[0] = rf / 256;
        buf[1] = rf % 256;
        if (rf < 100)
            msgid += MSG_FAULT;

        return limit_msg(mkhdr(msgid), 2, buf);

    case 30:
        msgid = MSG_SSM;

        if (dutyc_us*2 > period_us)  // less than 50%
            msgid += MSG_FAULT;
        return limit_msg(mkhdr(msgid), 2, buf);

    case 40:
        msgid = MSG_DEVFLT - MSG_GROUNDFLT;  // cheat..
        // fallthrough
    case 50:
        msgid += MSG_GROUNDFLT; // ...see!

        if ( ((100*dutyc_us) / period_us) < 47)
            break; 
        
        if ( ((100*dutyc_us) / period_us) > 53)
            break; 
        return limit_msg(mkhdr(msgid), 2, buf);
    }

    // not a valid frequency
    return limit_msg(mkhdr(MSG_INVALID), 0, buf);
}
static int period_us = 0; // time between start and end raising edges 
static int dutyc_us  = 0; // time between start raising and first falling edge
void TIM1_UP_IRQ_Handler(void) {
    serial_printf(&USART1, "%lld UP\n", cycleCount()/C_US);
    TIM1.SR &= ~TIM_SR_UIF;
    if (!period_us) {
        sendmsg(0, 0); // no complete period
    }
    period_us = 0;
    dutyc_us  = 0;        

    led0_on();
}

void TIM1_CC_IRQ_Handler(void) {
    uint16_t sr = TIM1.SR;

    if (sr & TIM_SR_CC1IF) {
        period_us = 4*(int)TIM1.CCR1;
        sendmsg(period_us, dutyc_us);
    }

    if (sr & TIM_SR_CC2IF) {
        dutyc_us = 4*(int)TIM1.CCR2;
        led0_off();
    }

    TIM1.SR &= ~(TIM_SR_CC1IF|TIM_SR_CC2IF);
}


int main(void) {
    uint8_t rf = (RCC.CSR >> 24) & 0xfc;
    RCC.CSR |= RCC_CSR_RMVF;  // Set RMVF bit to clear the reset flags

    SysTick_Config(1U << 24);  // tick at 72Mhz/2^24 = 4.2915 HZ

    NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);
    for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
        NVIC_SetPriority(irqprios[i].irq, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, irqprios[i].group, irqprios[i].sub));
    }

    RCC.APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
    RCC.APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_CAN1EN;

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

    bxcan_init(CAN_1MBd);

    // CC1,2 channel is configured as input, IC1,2 both mapped on TI1,  fSAMPLING=fDTS/8, N=8
    // TI1 is trigger, trigger is reset, CC2 captures duty, CC1 captures period
    TIM1.CR1    = 2 << 8;  // CKD = /4, 18MHz
    TIM1.DIER   = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
    TIM1.PSC    = (CLOCKSPEED_HZ / 250000) - 1;  // 250Khz or 4us.
    TIM1.ARR    = 0xffff;  // overflow every 262.144ms
    TIM1.SMCR   = (0b101 << 4) | 0b100;  //  TS = 101 (TI1 is trigger), SMS = 100 (trigger is reset)
    TIM1.CCMR1  = (9 << 12) | (9 << 4) | (2 << 8) | 1;
    TIM1.CCMR2  = 0;
    TIM1.CCER   = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;  // 1,2 enabled, 2 inverted
    TIM1.CR1   |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    // For debugging, generate 10Hz/33% on TIM3CH1 PA6
    TIM3.PSC    = (CLOCKSPEED_HZ / 10000) - 1;  // 10 Khz.
    TIM3.ARR    = 1000; // 10Hz  
    TIM3.CCMR1  = 0b110 << 4;  // PWM1 mode
    TIM3.CCR1   =  333; // 33% duty cycle
    TIM3.CCER   = TIM_CCER_CC1E;
    TIM3.CR1   |= TIM_CR1_CEN;

#if 0
    // Initialize the independent watchdog
    while (IWDG.SR != 0)
        __NOP();

    IWDG.KR = 0x5555;  // enable watchdog config
    IWDG.PR = 0;  // prescaler /4 -> 10kHz
    IWDG.RLR = 0xFFF;  // count to 4096 -> 409.6ms timeout
    IWDG.KR = 0xCCCC;  // start watchdog countdown

    //...
    IWDG.KR = 0xAAAA;
#endif

    serial_printf(&USART1, "%lld mainloop start\n", cycleCount()/C_US);

    for (;;)
        __WFI();

    return 0;
}
