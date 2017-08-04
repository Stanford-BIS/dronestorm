import time, sys, redis, pigpio
import numpy as np
import RPi.GPIO as GPIO
from DroneControl import Reader

YAW_CHN = 27
ROLL_CHN = 22
PITCH_CHN = 17
AUX1_CHN = 26
THR_CHN = 13

host = '127.0.0.1'
r = redis.StrictRedis(host=host)

pi = pigpio.pi()
ro = Reader(pi, ROLL_CHN)
p = Reader(pi, PITCH_CHN)
y = Reader(pi, YAW_CHN)
th = Reader(pi, THR_CHN)
aux = Reader(pi, AUX1_CHN)
time.sleep(0.1)

def measurePWM(pigpio_pulse):
    return pigpio_pulse.pulse_width()

try:
    while(True):
        yaw = measurePWM(y)
        pitch = measurePWM(p)
        roll = measurePWM(ro)
        thr = measurePWM(th)
        aux1 = measurePWM(aux)

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
    # Graceful Exit
    ro.cancel()
    p.cancel()
    y.cancel()
    aux.cancel()
    th.cancel()
    pi.stop()
