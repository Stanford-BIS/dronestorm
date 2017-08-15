import RPi.GPIO as GPIO
import time, sys, redis, pigpio
from dronestorm import DroneComm
from dronestorm import Reader

host = '127.0.0.1'
r = redis.StrictRedis(host=host)

FRONT_TRIG_ID = 12
FRONT_ECHO_ID = 16

BACK_TRIG_ID = 23
BACK_ECHO_ID = 24

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
MAX_DIST = 300
roll_trim = -4 
pitch_trim = 14

drone = DroneComm(roll_trim=roll_trim, pitch_trim=pitch_trim)

# Start program by placing drone on a flat surface to ensure accurate
# calibration values
# drone.update_attitude()
# desired_roll = drone.attitude['roll']
# desired_pitch = drone.attitude['pitch']
desired_roll = 0.
desired_pitch = 0.

print("Setting desired roll/pitch...")
time.sleep(1)

pi = pigpio.pi()
ro = Reader(pi, ROLL_CHN)
p = Reader(pi, PITCH_CHN)
y = Reader(pi, YAW_CHN)
th = Reader(pi, THR_CHN)
aux = Reader(pi, AUX1_CHN)
time.sleep(0.1)

def init_HCSRO4_Sonar(TRIG_ID, ECHO_ID):

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_ID, GPIO.OUT)
    GPIO.setup(ECHO_ID, GPIO.IN)
    GPIO.output(TRIG_ID, False)

    print("Init Sonar...")
    time.sleep(.5)

def measureDistance_HCSRO4(TRIG_ID, ECHO_ID):
    GPIO.output(TRIG_ID, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_ID, False)

    edge_detect = GPIO.wait_for_edge(ECHO_ID, GPIO.RISING, timeout = 100)

    if edge_detect is not None:
        pulse_start = time.time()
    else: return MAX_DIST

    edge_detect = GPIO.wait_for_edge(ECHO_ID, GPIO.FALLING, timeout = 100)

    if edge_detect is not None:
        pulse_end = time.time()
    else: return MAX_DIST

    pulse_duration = pulse_end - pulse_start
    distance = round(pulse_duration * 17150, 2)

    return distance

def measurePWM(pigpio_pulse):
    return pigpio_pulse.pulse_width()

init_HCSRO4_Sonar(FRONT_TRIG_ID, FRONT_ECHO_ID)
init_HCSRO4_Sonar(BACK_TRIG_ID, BACK_ECHO_ID)

try:
    while (True):
        front_dist = measureDistance_HCSRO4(FRONT_TRIG_ID, FRONT_ECHO_ID)
        back_dist = measureDistance_HCSRO4(BACK_TRIG_ID, BACK_ECHO_ID)
   
        # user's manual input
        yaw = measurePWM(y)
        pitch = measurePWM(p)
        roll = measurePWM(ro)
        thr = measurePWM(th)
        aux1 = measurePWM(aux)

        if ((front_dist <= desired_distance) and (back_dist <= desired_distance)) or ((front_dist > desired_distance) and (back_dist > desired_distance)):

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

        elif (front_dist <= desired_distance):
            pitch = K_front * front_dist + 0.0011

        elif (back_dist <= desired_distance):
            pitch = K_back * back_dist + 0.0019

        r.set('a_roll', roll)
        r.set('a_pitch', pitch)
        r.set('a_yaw', yaw)
        r.set('a_thr', thr)
        r.set('a_aux1', aux1)

        sys.stdout.write(
           "roll:%.5f pitch:%.5f yaw:%.5f thr:%.5f aux1:%.5f Front Distance:%4.2f Back Distance:%4.2f\r"%
           (roll, pitch, yaw, thr, aux1, front_dist, back_dist))
        sys.stdout.flush()

        time.sleep(0.1)

except (KeyboardInterrupt, SystemExit):
    ro.cancel()
    p.cancel()
    y.cancel()
    aux.cancel()
    th.cancel()
    pi.stop()
    GPIO.cleanup()
