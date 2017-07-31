import RPi.GPIO as GPIO
from RPIO import PWM
import time

def initSonar():
    GPIO.setmode(GPIO.BCM)

    TRIG_ID = 23
    ECHO_ID = 24

    GPIO.setup(TRIG_ID, GPIO.OUT)
    GPIO.setup(ECHO_ID, GPIO.IN)

    GPIO.output(TRIG_ID, False)
    time.sleep(2)

def measureDistance():
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


try:
    while (True):
        dist = measureDistance()
        print "Distance:",dist,"cm"
        time.sleep(0.4)

except (KeyboardInterrupt, SystemExit):
    GPIO.cleanup()
