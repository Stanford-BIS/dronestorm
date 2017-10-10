"""This class defines a class for the Maxbotix sonar"""
import struct
import pigpio

SONAR_STATUS_PIN = 4
I2C_SDA_PIN = 2
I2C_SCL_PIN = 3
HIGH_BYTE_MASK = 0b00000011

class SonarMaxbotix(object):
    """Class for running the Maxbotix 1242 I2CXL sonar module

    Connections:
        sonar's I2C pins to the RPi's I2C pins
        sonar's status pin to a RPi GPIO pin
                sonar pin | rpi pin
        ----------------------------------------
        i2c scl         5 | GPIO 3 / board pin 5
        i2c sda         4 | GPIO 2 / board pin 3
        status          2 | GPIO 4 / board pin 7

    Uses I2C pins for reading data
    Uses the status pin to time the measure/data requests
    """
    def __init__(self, i2c_address=0x70):
        self.pi_pins = pigpio.pi()
        self.pi_pins.set_mode(SONAR_STATUS_PIN, pigpio.INPUT)
        self.pi_pins.set_pull_up_down(SONAR_STATUS_PIN, pigpio.PUD_OFF)
        self.pi_pins.set_pull_up_down(I2C_SDA_PIN, pigpio.PUD_UP)
        self.pi_pins.set_pull_up_down(I2C_SCL_PIN, pigpio.PUD_UP)
        self.i2c = self.open_i2c(i2c_address)

    def close_i2c(self):
        """Close the I2C port for communication"""
        self.pi_pins.i2c_close(self.i2c)

    def open_i2c(self, i2c_address):
        """Open the I2C port for communication

        Returns a handle for the device at the given I2C address
        """
        return self.pi_pins.i2c_open(1, i2c_address)

    def _wait_for_ready(self):
        """Wait for status pin to go low"""
        while self.pi_pins.read(SONAR_STATUS_PIN) > 0:
            self.pi_pins.wait_for_edge(SONAR_STATUS_PIN, pigpio.FALLING_EDGE, .1)

    def read(self):
        """Report the distance to closest object in cm"""
        self._wait_for_ready()
        self.pi_pins.i2c_write_byte(self.i2c, 0x51)

        self._wait_for_ready()
        ret = self.pi_pins.i2c_read_device(self.i2c, 2)

        val = ret[1]
        high_byte, low_byte = struct.unpack('>BB', val)
        high_byte &= HIGH_BYTE_MASK
        val = high_byte << 8 | low_byte
        return val
