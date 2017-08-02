import sys
import time
import numpy as np
import RPi.GPIO as GPIO

FREQ = 45.45
T = 0.022
PW = 0.0015
D = PW/T * 100

GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.OUT)
pulse = GPIO.PWM(25, D)

pulse.start(D)
pulse.ChangeFrequency(FREQ)

channel=16
GPIO.setup(channel, GPIO.IN)

N = 25

########################### poll based timing
dt = np.zeros(N)
dt_idx = 0
trise = time.time()
tfall = time.time()
prev_state = GPIO.input(channel)
try:
    while(dt_idx<N):
        curr_state = GPIO.input(channel)
        if curr_state != prev_state and prev_state == GPIO.HIGH:
            tfall = time.time()
            prev_state = curr_state
            dt[dt_idx] = tfall-trise
            dt_idx += 1
        elif curr_state != prev_state and prev_state == GPIO.LOW:
            trise = time.time()
            prev_state = curr_state

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    print("Exiting")
    GPIO.cleanup()
    sys.exit()

print("MEAN: " + str(np.mean(dt)))
pulse.stop()
GPIO.cleanup()
