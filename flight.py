from __future__ import division
from pyMultiwii import MultiWii
import Adafruit_PCA9685, time

# channel 1 -> Yaw
# channel 2 -> Pitch
# channel 3 -> Roll

# range of values: 1.1ms -> 1.9ms

pwm = Adafruit_PCA9685.PCA9685()
board = MultiWii("/dev/ttyUSB0")

pwm.set_pwm_freq(1/.023)    # ~45.45 Hz

# used to adjust pulse width
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 1/.022       # ~45.45 Hz
    pulse_length //= 4096     # 12 bits of resolution
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, int(pulse))

# used to reset channels
def reset():
    for i in range(6):
        set_servo_pulse(i, 1.4)

SCALING_FACTOR = 1.4 / 1.5
MIN_WIDTH = 1.10
MAX_WIDTH = 1.90

def convertWidth(width):
    return width*SCALING_FACTOR

def setYaw(width):
    set_servo_pulse(1, convertWidth(width))

def setPitch(width):
    set_servo_pulse(2, convertWidth(width))

def setRoll(width):
    set_servo_pulse(3, convertWidth(width))

def getAngX():
    board.getData(MultiWii.ATTITUDE)
    return board.attitude['angx']

def getAngY():
    board.getData(MultiWii.ATTITUDE)
    return board.attitude['angy']

def getHeading():
    board.getData(MultiWii.ATTITUDE)
    return board.attitude['heading']

print(getHeading())
