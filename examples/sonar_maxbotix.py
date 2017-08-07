"""Examples script to read from a maxbotix 1242 sonar module over i2c

Connections:
    sonar's I2C pins to the RPi's I2C pins
    sonar's status pin to a RPin GPIO pin
            sonar pin | rpi pin
    ----------------------------------------
    i2c scl         5 | GPIO 3 / board pin 5
    i2c sda         4 | GPIO 2 / board pin 3
    status          2 | GPIO 4 / board pin 7

Uses i2c pins for commuincation
Uses the status pin to time the measure/data requests
"""
import pigpio
import struct

pi = pigpio.pi()
sonar_status_pin = 4
pi.set_mode(sonar_status_pin, pigpio.INPUT) # 
h = pi.i2c_open(1, 0x70)

def wait_for_ready():
    # wait for status pin to go low
    while pi.read(sonar_status_pin) > 0:
        pi.wait_for_edge(sonar_status_pin, pigpio.FALLING_EDGE, .1)

try:
    while True:
        wait_for_ready()
        pi.i2c_write_byte(h, 0x51)

        wait_for_ready()
        ret = pi.i2c_read_device(h, 2)

        val = ret[1]
        high_byte, low_byte = struct.unpack('>BB', val)
        high_byte &= 0b00000011
        val = high_byte << 8 | low_byte
        print("high_byte:%u low_byte:%u value:%u"%(high_byte, low_byte, val))
except KeyboardInterrupt:
    pi.i2c_close(h)
    print ""
