# stm32f103_ir155tocan
Decode Bender Isometer IR155 PWM output and send as CAN messages

https://www.bender.de/en/products/insulation-monitoring/isometer-ir155-3203-ir155-3204/

The PWM signal is sampled on PA8 (TIM1 CH1).

Depending on frequency and duty cycle the following CAN messages are sent:

No Signal 
Normal Operation (10Hz)  Insulation resistor value in kOhm uint16 big endian 0..50000
Undervoltage     (20Hz)  Insulation resistor value in kOhm uint16 big endian 0..50000 
Speed Start Measurement (30Hz) Good/Bad
Device Error     (40Hz)
Ground Fault     (50Hz)
Invalid signal  (frequency/duty cycle out of bounds): period, duty cycle uint16 in units of 4us.

Duplicate messages within 100ms are supressed, but changes are broadcast without delay.
