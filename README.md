# stm32f103_ir155tocan
Decode Bender Isometer IR155 PWM output and send as CAN messages

https://www.bender.de/en/products/insulation-monitoring/isometer-ir155-3203-ir155-3204/

The PWM signal is sampled on PA8 (TIM1 CH1).

Depending on frequency and duty cycle different CAN messages are sent.
The header is composed as 0x1969x908  with x the message id.
The message ID consists of the following flags:

| FLAG | NAME | Semantics |
|----  |---|--------------|
| 0x1 | NORMALOP   | payload present, resistance in kOhm big endian uint16 |
| 0x2 | UNDERVOLT  | combined with normal op: voltage below 500V |
| 0x4 | SSM        | speed start measurement mode |
| 0x8 | FAULT      | something's wrong |

They are combined as follows:

| ID   | Freq | DC | NAME | Payload | Semantics |
|----  |------|--- |-----|-------|--------------|
|  0x1 | 10   | <45% | NORMALOP    | uint16 | resistance in kOhm > 100k |
|  0x2 |      |      | RESERVED2   |        | not used
|  0x3 | 20   |      | NORMAL_UV   | uint16 | resistance in kOhm > 100k, low voltage |
|  0x4 | 30   | <10% | SSM         |        | speed start measurement mode OK
|  0x8 |      |      | NO_SIGNAL   |        |  No pwm signal detected
|  0x9 | 10   | >45% | INSFAULT    | uint16  | Insulation below critical value
|  0xA |      | ~50% | GROUNDFLT   |        | Ground Fault
|  0xB | 20   | >45% | INSFAULT_UV | uint16 | Insulation below critical value + low voltage |
|  0xC | 30   | >90% | INSFAULT_SSM|        | speed start measurement mode FAULT
|  0xD |      |      | RESERVEDD   |        | not used
|  0xE |      | ~50% | DEVFLT      |        | Device fault 
|  0xF |      |      | INVALID     |        | outside of valid period/dutyc bounds |

Duplicate messages within 90ms are supressed, but changes are broadcast without delay.
