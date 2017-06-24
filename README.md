# DroneControl

This library can be used to program flight controllers and acquire telemetry data. The applications of this allows one to program flight stabilization programs, autonomy algorithms and object recognition suites.

DroneControl is meant to be implemented on an onboard computer such as a Raspberry pi, and sends commands to a flight controller via the pi's I2C interface.

## Hardware

* Rasbperry pi 3 - onboard computer
* [Adafruit PWM/Servo 8 channel featherwing](https://learn.adafruit.com/adafruit-8-channel-pwm-or-servo-featherwing/assembly) - PWM output board
* Naze32 - Flight controller
