import RPi.GPIO as GPIO
import time, sys, redis, pigpio, struct
from dronestorm import DroneComm
from dronestorm import Reader

host = '127.0.0.1'
r = redis.StrictRedis(host=host)

YAW_CHN = 27
ROLL_CHN = 22
PITCH_CHN = 17
AUX1_CHN = 26
THR_CHN = 13

# Proportion coefficients: how strongly error should be corrected
K_roll  = 4 * 1./180.
K_pitch = 4 * 1./90.

desired_distance = 100. # cm
K_front = 0.0004 / 100 # proportion constant
K_back = -0.0004 / 100

MID_WIDTH = 0.00150
MAX_DELTA_PWIDTH = 0.0004
roll_trim =  0
pitch_trim = 0

drone = DroneComm(roll_trim=roll_trim, pitch_trim=pitch_trim)

# Start program by placing drone on a flat surface to ensure accurate
# calibration values
drone.update_attitude()
desired_roll = drone.attitude['roll']
desired_pitch = drone.attitude['pitch']

print("Setting desired roll/pitch...")
time.sleep(1)

pi = pigpio.pi()
sonar_status_pin = 4
i2c_sda_pin = 2
i2c_scl_pin = 3
pi.set_mode(sonar_status_pin, pigpio.INPUT)
pi.set_pull_up_down(sonar_status_pin, pigpio.PUD_OFF)
pi.set_pull_up_down(i2c_sda_pin, pigpio.PUD_UP)
pi.set_pull_up_down(i2c_scl_pin, pigpio.PUD_UP)
h = pi.i2c_open(1, 0x71)

ro = Reader(pi, ROLL_CHN)
p = Reader(pi, PITCH_CHN)
y = Reader(pi, YAW_CHN)
th = Reader(pi, THR_CHN)
aux = Reader(pi, AUX1_CHN)
time.sleep(0.1)

def wait_for_ready():
    # wait for status pin to go low
    while pi.read(sonar_status_pin) > 0:
        pi.wait_for_edge(sonar_status_pin, pigpio.FALLING_EDGE, .1)

def getDistMaxSonar():
    wait_for_ready()
    pi.i2c_write_byte(h, 0x51)
    wait_for_ready()
    ret = pi.i2c_read_device(h, 2)
    val = ret[1]
    high_byte, low_byte = struct.unpack('>BB', val)
    high_byte &= 0b00000011
    val = high_byte << 8 | low_byte

    return val

def measurePWM(pigpio_pulse):
    return pigpio_pulse.pulse_width()

try:
    prev_thr = 0.0011

    while (True):

        desired_height = getDistMaxSonar()

        # user's manual input
        yaw = measurePWM(y)
        pitch = measurePWM(p)
        roll = measurePWM(ro)
        thr = measurePWM(th)
        aux1 = measurePWM(aux)

        # calculates stable values
        drone.update_attitude()
        curr_roll = drone.attitude['roll']
        curr_pitch = drone.attitude['pitch']

        # Error between desired and actual roll/pitch
        error_roll =  desired_roll - curr_roll
        error_pitch = desired_pitch - curr_pitch
        output_roll_rate = K_roll * error_roll
        output_pitch_rate = K_pitch * error_pitch

        pitch = MID_WIDTH + (pitch_trim * 1E-6) + (output_pitch_rate * MAX_DELTA_PWIDTH)
        roll = MID_WIDTH + (roll_trim * 1E-6) + (output_roll_rate * MAX_DELTA_PWIDTH)

        curr_height = getDistMaxSonar()

        if curr_height != desired_height:

            thr = (((0.00190 - thr) / (700 - curr_height)) * (x - 700)) + 0.00190


        r.set('a_roll', roll)
        r.set('a_pitch', pitch)
        r.set('a_yaw', yaw)
        r.set('a_thr', thr)
        r.set('a_aux1', aux1)

        sys.stdout.write(
            "roll:%.5f pitch:%.5f yaw:%.5f thr:%.5f aux1:%.5f Front Distance:%4.2f Back Distance:%4.2f\r"%
            (roll, pitch, yaw, thr, aux1, front_dist, back_dist))

        prev_thr = thr
        sys.stdout.flush()
        time.sleep(0.01)

except (KeyboardInterrupt, SystemExit):
    ro.cancel()
    p.cancel()
    y.cancel()
    aux.cancel()
    th.cancel()
    pi.stop()
    pi.i2c_close(h)
    GPIO.cleanup()
