# DroneControl

This library can be used to program flight controllers and acquire telemetry
data. The applications of this allows one to program flight stabilization
programs, autonomy algorithms and object recognition suites.

DroneControl is meant to be implemented on an onboard computer such as a
Raspberry pi, and sends commands to a flight controller via the pi's I2C
interface.

## Hardware

* Rasbperry pi 3 - Onboard computer
* [Adafruit PWM/Servo 8 channel featherwing](https://learn.adafruit.com/adafruit-8-channel-pwm-or-servo-featherwing/assembly) - PWM output board
* [Pololu 4 channel RC servo multiplexer](https://www.pololu.com/product/2807) - RC switch
* Naze32 - Flight controller

## I2C Interface

The onboard communicates with the PWM output board via the I2C protocol.
As described [here](https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c),
check  the pi's I2C interface using
```
sudo i2cdetect -y 1
```

The I2C library is provided by the [ext/Adafruit_Python_PCA9685](https://github.com/adafruit/Adafruit_Python_PCA9685) subtree.
Note that it must be installed per the README directions within the ext/Adafruit_Python_PCA9685 directory.

Execute register_startup.sh to set up startup.py to run upon boot.

# Installation

TODO: make this a script

* install adafruit i2c library
* register startup script
