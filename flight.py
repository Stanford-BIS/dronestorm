from __future__ import division
from pyMultiwii import MultiWii
import Adafruit_PCA9685, time

# channel 1 -> Yaw
# channel 2 -> Pitch
# channel 3 -> Roll

# range of values: 1.1ms -> 1.9ms

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(1/.023)    # ~45.45 Hz

# used to adjust pulse width
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 1/.022       # ~45.45 Hz
    pulse_length //= 4096     # 12 bits of resolution
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, int(pulse))

# used to reset channels
def reset():
    for i in range(6):
        set_servo_pulse(i, 1.4)

SCALING_FACTOR = 1.4 / 1.5
MIN_WIDTH = 1.10
MAX_WIDTH = 1.90

def convertWidth(width):
    return width*SCALING_FACTOR

def setYaw(width):
    width_correx, valid = isValid(width)

    set_servo_pulse(1, width_correx)

    if not valid:
        print("Warning Yaw out of range!")

def setPitch(width):
    width_correx, valid = isValid(width)

    set_servo_pulse(2, width_correx)

    if not valid:
        print("Warning Pitch out of range!")

def setRoll(width):
    width_correx, valid = isValid(width)

    set_servo_pulse(3, width_correx)

    if not valid:
        print("Warning Roll out of range!")

def isValid(width):
    width_c = convertWidth(width)

    if(width_c > 1.9):
        return convertWidth(1.9), False
    elif(width_c < 1.1):
        return convertWidth(1.1), False
    else:
        return width_c, True

def getData(board, arg):
    board.getData(MultiWii.ATTITUDE)

    if arg == 'angx': # Roll
        x = board.attitude[arg]
        return x
    elif arg == 'angy': # Pitch
        x = board.attitude[arg]
        return x
    elif arg == 'heading': # Yaw
        x = board.attitude[arg]
        return x
    else:
        print("Invalid argument\n")
        board.closeSerial()

def convertIntoMotor(output):
    return int(2.168*output + 301.5)

def servoTest():

    error_p = integral = error = 0
    elapsed_time = 0
    timex = 0.001

    desired_Pitch = 0

    K_pitch = 1
    K_i = 0
    K_d = 0

    while (True):
        x = getData("angy")
        print("Desired Pitch: " + str(desired_Pitch))
        print("Current Pitch: " + str(x))
        error = x - desired_Pitch

        integral += error*timex
        derivative = (error - error_p) / timex
        output = K_pitch*error + K_i*integral + K_d*derivative

        error_p = error

        print("Suggested Output: " + str(output) + "\n")

        pwm.set_pwm(7, 0, convertIntoMotor(output*-1))

        time.sleep(timex)

def flightStabilization(des_p, des_r):
    reset()
    time.sleep(1)
    board = MultiWii("/dev/ttyUSB0")

    K_pitch = .01
    K_roll = .01

    desired_Pitch = des_p
    desired_Roll = des_r

    try:
        while (True):
            pitch = getData(board, "angy")
            roll = getData(board, "angx")

            error_pitch = desired_Pitch - pitch
            error_roll =  desired_Roll - roll

            output_pitch = K_pitch * error_pitch + pitch
            output_roll = K_roll * error_roll + roll

            setPitch(output_pitch)
            setRoll(output_roll)
            setYaw(1.5)


    except (KeyboardInterrupt, SystemExit):
        board.closeSerial()

# RC Multiplexer
reset()
flightStabilization(0,0)
