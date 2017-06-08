from __future__ import division
import Adafruit_PCA9685, time

pwm = Adafruit_PCA9685.PCA9685()

# pwm_min = 223
# pwm_max = 335
#
# pwm.set_pwm_freq(43)
#
# #while True:
# pwm.set_pwm(2, 0, pwm_max)

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 1/.022       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    print(pulse)
    pwm.set_pwm(channel, 0, int(pulse))

pwm.set_pwm_freq(1/.023)

for i in range(6):
    print(i)
    set_servo_pulse(i, 1.4)
