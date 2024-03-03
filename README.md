Fan control with Microchip PIC12F615 and 1-wire BUS sensor Dallas DS18B20.

This is a Microchip PIC12F615 with PWM at 10 Khz acting on various duty cycles,
0%, 15%, 30%, 45%, 60%, 75%, 90% and 100% depending on the temperature reading 
from 1-wire BUS Dallas DS18B20 temperature sensor.

* CONFIGURATION OF GPIO PINS
------------------------------
* GP2 - PWM OUTPUT P1A - PIN 5
* GP4 - DS18B20        - PIN 3
* GP5 - OUTPUT LED     - PIN 2 

Temperature and duty cycle
------------------------------
* 30C - 34C duty cycle  15%
* 35C - 39C duty cycle  30%
* 40C - 44C duty cycle  45%
* 45C - 49C duty cycle  60%
* 50C - 54C duty cycle  75%
* 55C - 59C duty cycle  90%
* 60C -     duty cycle 100%
