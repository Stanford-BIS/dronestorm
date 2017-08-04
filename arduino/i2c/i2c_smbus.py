"""Example to run on the Raspberry Pi for I2C communication with the Arduino

Uses the smbus Python library
"""
import smbus
import time

i2cbus = 1 # 1 except with older model; check with i2cdetect -y [0, 1]
address = 0x08 # address we setup in the Arduino Program
bus = smbus.SMBus(i2cbus)

for i in range(10):
    bus.write_byte(address, i)

for i in range(10):
    print(bus.read_byte(address))
