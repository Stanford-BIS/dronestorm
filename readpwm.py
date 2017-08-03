import time, sys, redis
import numpy as np
import RPi.GPIO as GPIO

YAW_CHN = 27
ROLL_CHN = 22
PITCH_CHN = 17
AUX1_CHN = 26
THR_CHN = 13

GPIO.setmode(GPIO.BCM)

host = '127.0.0.1'
r = redis.StrictRedis(host=host)

def measurePWM(channel):
    GPIO.setup(channel, GPIO.IN)
    trise = time.time()
    tfall = time.time()
    prev_state = GPIO.input(channel)

    N = 0
    tot = 0

    while(True):
        curr_state = GPIO.input(channel)
        if curr_state != prev_state and prev_state == GPIO.HIGH:
            tfall = time.time()
            prev_state = curr_state
            N += 1

            if N == 1 :
                tot += tfall - trise
            elif N == 2:
                tot += tfall - trise
                N = 0
                temp = tot
                tot = 0
                return temp / 2

        elif curr_state != prev_state and prev_state == GPIO.LOW:
            trise = time.time()
            prev_state = curr_state

try:
    while(True):
        yaw = measurePWM(YAW_CHN)
        pitch = measurePWM(PITCH_CHN)
        roll = measurePWM(ROLL_CHN)
        thr = measurePWM(THR_CHN)
        aux1 = measurePWM(AUX1_CHN)

        r.set('roll', roll)
        r.set('pitch', pitch)
        r.set('yaw', yaw)
        r.set('thr', thr)
        r.set('aux1', aux1)

        sys.stdout.write(
            "roll:%.5f pitch:%.5f yaw:%.5f thr:%.5f aux1:%.5f\r"%
            (roll, pitch, yaw, thr, aux1))
        sys.stdout.flush()
except (KeyboardInterrupt, SystemExit):
    GPIO.cleanup()
