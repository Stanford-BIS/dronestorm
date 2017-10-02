# dronestorm

This library can be used to program flight controllers and acquire telemetry
data. The applications of this allows one to program flight stabilization
programs, autonomy algorithms and object recognition suites.

dronestorm is meant to be run on an onboard computer such as a
Raspberry Pi, and sends commands to a flight controller via the pi's I2C
interface.

# Quadcopter Build 2

In build 2, we do away with the PWM signals between the receiver, the Raspberry Pi, 
and the flight control board. 

## Parts

* [Flite Test Versacopter 2](https://store.flitetest.com/ft-versacopter-280-v2-quad-kit/) - chassis
* [Makerfire BLHeli_S ESC](https://www.amazon.com/Makerfire-Electronic-Controller-Multicopter-Quadcopter/dp/B01MXJNGQD) - electronic speed controllers (AKA DC brushless motor controllers)
* [Lumenier 2206 2350kv motors](https://store.flitetest.com/lumenier-rx2206-11-2350kv-motor) - DC brushless motors
* [Rasbperry Pi 3](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) - onboard general purpose computer
* [Omnibus F3 pro](http://www.readytoflyquads.com/flip-32-f3-omnibus-pro)- flight control board (onboard microcontroller)
* [Spektrum remote receiver](http://spektrumrc.com/Products/Default.aspx?ProdID=SPM9645) - RC receiver
* [Maxbotix I2CXL sonar](https://www.maxbotix.com/Ultrasonic_Sensors/MB1242.htm) - high quality sonar sensor
* [HC-SR04 sonar](https://www.amazon.com/dp/B0716SSPR5) - low quality sonar sensor

## Installation

`sudo python setup.py develop`

## Run

Runtime scripts can be found in the `runtime` directory. Executing one of the scripts should start
the appropriate modules from the `runtime_modules` directory. 
As an example, to run a controller which simply forwards receiver data to
the flight controller, change to the `runtime` directory and run `./run_control_none.sh`

# Quadcopter Build 1

In this build, the Raspberry Pi takes data in from the flight control board via
the USB port and sends commands to a flight control board via the Pi's I2C
interface to the PWM output generator

## Parts

* [Flite Test Versacopter 2](https://store.flitetest.com/ft-versacopter-280-v2-quad-kit/) - chassis
* [Lumenier BL Heli ESC](https://store.flitetest.com/e-pack-2206-lumenier-2000kv-best-2/) - electronic speed controllers (AKA DC brushless motor controllers)
* [Lumenier 2206 2000kv motors](https://store.flitetest.com/e-pack-2206-lumenier-2000kv-best-2/) - DC brushless motors
* [Rasbperry Pi 3](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) - onboard general purpose computer
* [Naze32](https://www.amazon.com/Naze-Acro-Flight-Controller-Abusemark/dp/B015NK80J4) - flight control board (onboard microcontroller)
* [Spektrum AR610](http://spektrumrc.com/Products/Default.aspx?ProdID=SPMAR610) - RC receiver
* [Adafruit PWM/Servo 8 channel featherwing](https://learn.adafruit.com/adafruit-8-channel-pwm-or-servo-featherwing/assembly) - PWM output board
* [Pololu 4 channel RC servo multiplexer](https://www.pololu.com/product/2807) - RC switch
* [Maxbotix I2CXL sonar](https://www.maxbotix.com/Ultrasonic_Sensors/MB1242.htm) - high quality sonar sensor
* [HC-SR04 sonar](https://www.amazon.com/dp/B0716SSPR5) - low quality sonar sensor

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

## Installation

`sudo python setup.py develop`

* install adafruit i2c library
* register startup script
