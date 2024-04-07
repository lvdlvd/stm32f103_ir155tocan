# stm32f103_ir155tocan
Decode Bender Isometer IR155 PWM output and send as CAN messages

https://www.bender.de/en/products/insulation-monitoring/isometer-ir155-3203-ir155-3204/

The PWM signal is sampled on PA8 (TIM1 CH1).

Depending on frequency and duty cycle the following CAN messages are sent:


| ID   | Freq| DC | NAME | Payload | Semantics |
|----  |---|--- |-----|-------|--------------|
|  0x1 | 10/20 |      | NORMALOP     | uint16 | FLAG: payload present |
|  0x1 | 10    | <45% | NORMALOP     | uint16 | by itself: resistance in kOhm > 100k |
|  0x2 | 20    |      | UNDERVOLT    |        | flag: combined with normal op |
|  0x3 | 20    |      | NORMAL_UV    | uint16 |  resistance in kOhm > 100k |
|  0x4 | 30    |      | SSM          |        | flag speed start measurement mode
|  0x4 | 30    | <10% | SSM          |        | by itself speed start measurement mode OK
|  0x8 |       |      | FAULT        |        | flag: something's wrong
|  0x8 |       |      | NO_SIGNAL    |        |  by itself no pwm detected
|  0x9 | 10    | >45% | INSFAULT     |uint16  | Insulation below critical value
|  0xA |       | ~50% | GROUNDFLT    |        | Ground Fault
|  0xB | 20    | >45% | INSFAULT_UV  | uint16 | Insulation below critical value + undervoltage
|  0xC | 30    | >90% | INSFAULT_SSM |        | speed start measurement mode FAULT
|  0xD |       |      | RESERVED     |        | not used
|  0xE |       | ~50% | DEVFLT       |        | Device fault 
|  0xF |       |      | INVALID      |        | outside of valid period/dutyc bounds |

The header is composed as 0x1969x908  with x the message id.

TODO: Duplicate messages within 100ms are supressed, but changes are broadcast without delay.
