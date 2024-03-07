Fan control with Microchip PIC12F615, 1-wire BUS sensor Dallas DS18B20,
and n-channel mosfet IRLR/U2905PbF.

This is a Microchip PIC12F615 with PWM at 10 Khz acting on various duty cycles,
0%, 15%, 30%, 45%, 60%, 75%, 90% and 100% depending on the temperature reading 
from 1-wire BUS Dallas DS18B20 temperature sensor.

Logic Level N-CHANNEL MOSFET IRLR/U2905PbF is being used for the PWM gate pulse.
Continuous Drain Current, VGS @10V 42A MAX rating.
Gate-to-Source Voltage +-16V
Gate Threshold Voltage MIN 1V MAX 2V, VDS = VGS , ID = 250µA 
RG = 3.4Ω, VGS = 5.0V

* CONFIGURATION OF GPIO PINS
------------------------------
* GP2 - PWM OUTPUT P1A - PIN 5
* GP4 - DS18B20        - PIN 3
* GP5 - OUTPUT LED     - PIN 2 

Temperature and duty cycle
------------------------------
* 30°C - 34°C duty cycle  15%
* 35°C - 39°C duty cycle  30%
* 40°C - 44°C duty cycle  45%
* 45°C - 49°C duty cycle  60%
* 50°C - 54°C duty cycle  75%
* 55°C - 59°C duty cycle  90%
* 60°C -     duty cycle 100%

Pic is set with Internal Oscillator Frequency Select (8 MHz)
command cycle at 500 nano seconds.

The nmain.c file was created with MPLAB IDE v6.20, and the compiler used XC8-v2.46.
The HEX file can be used directly to programme the 12F615.
Programmer hardware used PICKIT 3.
