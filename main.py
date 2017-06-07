from __future__ import division
import smbus, Adafruit_PCA9685, time

pwm = Adafruit_PCA9685.PCA9685()

servo_min = 150
servo_max = 600

pwm.set_pwm_freq(60)

channel = input("Enter channel of servo: ")

cont = True

while cont:
    pwm.set_pwm(channel, 0, servo_min)
    time.sleep(1)
    pwm.set_pwm(channel, 0, servo_max)
    time.sleep(1)
