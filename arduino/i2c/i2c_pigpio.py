"""Example to run on the Raspberry Pi for I2C communication with the Arduino

Uses the pigpio Python library
"""
import pigpio
import time

i2cbus = 1 # 1 except with older model; check with i2cdetect -y [0, 1]
address = 0x08 # address we setup in the Arduino Program

pi = pigpio.pi()
arduino_comm = pi.i2c_open(i2cbus, 0x08)

for i in range(10):
    pi.i2c_write_byte(arduino_comm, i)

for i in range(10):
    print(pi.i2c_read_byte(arduino_comm))
