# demo script to output pwm signal
from RPi import GPIO
import time

pin = 4 # set this to pin used for pwm output
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)

period = 0.02
freq = 1./period
tgt_pwidth = 0.0011

p = GPIO.PWM(pin, freq)
p.start(0.5)
try:
    while True:
        tgt_pwidth = float(raw_input("input tgt_pwidth:"))
        dc = tgt_pwidth/period * 100. # duty cycle in percent
        p.ChangeDutyCycle(dc)
except:
    p.stop()
    GPIO.cleanup()
print("")
