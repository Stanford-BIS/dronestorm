import RPi.GPIO as GPIO
import time, sys
import numpy as np
from smbus import SMBus

TRIG_ID = 23
ECHO_ID = 24

def init_HCSRO4_Sonar():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(TRIG_ID, GPIO.OUT)
    GPIO.setup(ECHO_ID, GPIO.IN)

    GPIO.output(TRIG_ID, False)
    time.sleep(2)

def measureDistance_HCSRO4():
    GPIO.output(TRIG_ID, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_ID, False)

    while GPIO.input(ECHO_ID) == 0:
      pulse_start = time.time()

    while GPIO.input(ECHO_ID) == 1:
      pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = round(pulse_duration * 17150, 2)

    return distance

# init_HCSRO4_Sonar()
#
# try:
#     while (True):
#         dist = measureDistance_HCSRO4()
#         print "Distance:",dist,"cm"
#         time.sleep(0.2)
#
# except (KeyboardInterrupt, SystemExit):
#     GPIO.cleanup()

try:
    while True:
        i2cbus = SMBus(1)
        i2cbus.write_byte(0x70, 0x51)
        time.sleep(0.12)
        val = i2cbus.read_word_data(0x70, 0xe1)
        print (val >> 8) & 0xff * 256 + (val & 0xff)
        # print (val >> 8) & 0xff | (val & 0xff), 'cm'
except IOError, err:
  print err
