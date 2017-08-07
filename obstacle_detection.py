import RPi.GPIO as GPIO
import time, sys
import numpy as np
from smbus import SMBus

FRONT_TRIG_ID = 12
FRONT_ECHO_ID = 16

BACK_TRIG_ID = 23
BACK_ECHO_ID = 24

def init_HCSRO4_Sonar(TRIG_ID, ECHO_ID):
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(TRIG_ID, GPIO.OUT)
    GPIO.setup(ECHO_ID, GPIO.IN)

    GPIO.output(TRIG_ID, False)

    print("Init Sonar...")
    time.sleep(1)

def measureDistance_HCSRO4(TRIG_ID, ECHO_ID):
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

init_HCSRO4_Sonar(FRONT_TRIG_ID, FRONT_ECHO_ID)
init_HCSRO4_Sonar(BACK_TRIG_ID, BACK_ECHO_ID)

try:
    while (True):
        front_dist = measureDistance_HCSRO4(FRONT_TRIG_ID, FRONT_ECHO_ID)
        back_dist = measureDistance_HCSRO4(BACK_TRIG_ID, BACK_ECHO_ID)

        sys.stdout.write(
            "Front Distance:%6.2f Back Distance:%5.2f\r" %
            (front_dist, back_dist))

        time.sleep(0.5)
        sys.stdout.flush()

except (KeyboardInterrupt, SystemExit):
    GPIO.cleanup()
