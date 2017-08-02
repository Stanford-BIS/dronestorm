import time, sys
import numpy as np
import RPi.GPIO as GPIO
from DroneControl import comm

FREQ = 45.45
T = 0.022
PW = 0
D = PW/T * 100

GPIO.setmode(GPIO.BCM)

def measurePWM(channel):
    GPIO.setup(channel, GPIO.IN)

    N = 1

    dt = np.zeros(N)
    dt_idx = 0
    trise = time.time()
    tfall = time.time()
    prev_state = GPIO.input(channel)

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

    return np.mean(dt)

YAW_CHN = 27
ROLL_CHN = 22
PITCH_CHN = 17
AUX1_CHN = 26
THR_CHN = 13

try:
    while(True):
        yaw = measurePWM(YAW_CHN)
        pitch = measurePWM(PITCH_CHN)
        roll = measurePWM(ROLL_CHN)
        thr = measurePWM(THR_CHN)
        aux1 = measurePWM(AUX1_CHN)

        sys.stdout.write(
            "roll:%.5f pitch:%.5f yaw:%.5f thr:%.5f aux1:%.5f\r"%
            (roll, pitch, yaw, thr, aux1))
        sys.stdout.flush()
except (KeyboardInterrupt, SystemExit):
    GPIO.cleanup()
