from __future__ import division
from pyMultiwii import MultiWii
import Adafruit_PCA9685, time

"""
Library to acquire Telemetry Data from Naze32 board (angx, angy and heading).
Furthermore, contains methods to send Roll/Pitch/Yaw signals to the Naze32
flight controller from a Raspberry pi via a Feather board using PWM signals.
"""

class DroneControl:

    # Range of Values (Pulse Width): 1.1ms -> 1.9ms
    MIN_WIDTH = 1.10
    MAX_WIDTH = 1.90
    MED_WIDTH = 1.50

    # Featherboard channels
    YAW_CHANNEL = 1
    PITCH_CHANNEL = 2
    ROLL_CHANNEL = 3

    SCALING_FACTOR = 1.4 / 1.5

    def __init__(self):
        '''
        Initializes PWM object
        '''

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(1/.023)    # ~45.45 Hz

    def set_servo_pulse(channel, pulse):
        '''
        Used to set a certain Pulse Width on a channel
        '''
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 1/.022       # ~45.45 Hz
        pulse_length //= 4096     # 12 bits of resolution
        pulse *= 1000
        pulse //= pulse_length

        self.pwm.set_pwm(channel, 0, int(pulse))

    def convertWidth(width):
        '''
        Internal conversion method to send PWM signals to Adafruit library
        '''
        return width * SCALING_FACTOR

    def setYaw(width):
        '''
        Sets the Yaw to a desired PWM width
        '''
        width_c, valid = isValid(width)
        set_servo_pulse(YAW_CHANNEL, width_correx)

        if not valid:
            print("WARNING: Yaw out of range!")

    def setPitch(width):
        '''
        Sets the Pitch to a desired PWM width
        '''
        width_c, valid = isValid(width)
        set_servo_pulse(PITCH_CHANNEL, width_correx)

        if not valid:
            print("WARNING: Pitch out of range!")

    def setRoll(width):
        '''
        Sets the Roll to a desired PWM width
        '''
        width_c, valid = isValid(width)
        set_servo_pulse(ROLL_CHANNEL, width_correx)

        if not valid:
            print("WARNING: Roll out of range!")

    def isValid(width):
        '''
        Verifies that the PWM signals are in the accepted range.
        If not, the MAX_WIDTH or MIN_WIDTH is returned
        '''
        width_c = convertWidth(width)

        if(width_c > MAX_WIDTH):
            return convertWidth(MAX_WIDTH), False
        elif(width_c < MIN_WIDTH):
            return convertWidth(MIN_WIDTH), False
        else:
            return width_c, True

    def reset():
        '''
        Resets channels 0-6 on the feather board; used as cleanup measure
        '''
        for i in range(6):
            set_servo_pulse(i, MED_WIDTH)

    def getData(board, arg):
        '''
        Returns the Attitude telemetry data from the Naze32 flight controller

        Parameters: Board MultiWii Object, Desired data point {angx, angy, heading}
        Returns: double
        '''
        board.getData(MultiWii.ATTITUDE)

        if arg == 'angx' or arg == 'angy' or arg == 'heading':
            return board.attitude[arg]
        else:
            print("Invalid argument\n")
            board.closeSerial()

    def signalControlExample():
        '''
        Sets the Roll/Pitch/Yaw on the Naze32 flight controller
        to maximum then minimum pulse widths
        '''
        reset()
        time.sleep(1)

        setYaw(MAX_WIDTH)
        setPitch(MAX_WIDTH)
        setRoll(MAX_WIDTH)

        time.sleep(2)

        setYaw(MIN_WIDTH)
        setPitch(MIN_WIDTH)
        setRoll(MIN_WIDTH)

        time.sleep(2)
        reset()
