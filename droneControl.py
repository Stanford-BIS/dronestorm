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

    def set_servo_pulse(self, channel, pulse):
        '''
        Used to set a certain Pulse Width on a channel
        '''
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 1/.022       # ~45.45 Hz
        pulse_length //= 4096     # 12 bits of resolution
        pulse *= 1000
        pulse //= pulse_length

        self.pwm.set_pwm(channel, 0, int(pulse))

    def convertWidth(self, width):
        '''
        Internal conversion method to send PWM signals to Adafruit library
        '''
        return width * self.SCALING_FACTOR

    def setYaw(self, width):
        '''
        Sets the Yaw to a desired PWM width
        '''
        width_c, valid = self.isValid(width)
        self.set_servo_pulse(self.YAW_CHANNEL, width_c)

        if not valid:
            print("WARNING: Yaw out of range!")

    def setPitch(self, width):
        '''
        Sets the Pitch to a desired PWM width
        '''
        width_c, valid = self.isValid(width)
        self.set_servo_pulse(self.PITCH_CHANNEL, width_c)

        if not valid:
            print("WARNING: Pitch out of range!")

    def setRoll(self, width):
        '''
        Sets the Roll to a desired PWM width
        '''
        width_c, valid = self.isValid(width)
        self.set_servo_pulse(self.ROLL_CHANNEL, width_c)

        if not valid:
            print("WARNING: Roll out of range!")

    def isValid(self, width):
        '''
        Verifies that the PWM signals are in the accepted range.
        If not, the MAX_WIDTH or MIN_WIDTH is returned
        '''

        if(width > self.MAX_WIDTH):
            return self.convertWidth(self.MAX_WIDTH), False
        elif(width < self.MIN_WIDTH):
            return self.convertWidth(self.MIN_WIDTH), False
        else:
            return self.convertWidth(width), True

    def reset(self):
        '''
        Resets channels 0-6 on the feather board; used as cleanup measure
        '''
        for i in range(6):
            self.set_servo_pulse(i, self.convertWidth(self.MED_WIDTH))

    def getData(self, board, arg):
        '''
        Returns the Attitude telemetry data from the Naze32 flight controller

        Parameters: Board MultiWii Object, Desired data point {angx, angy, heading}
        Returns: double
        '''
        self.board = MultiWii("/dev/ttyUSB0")
        self.board.getData(MultiWii.ATTITUDE)

        if arg == 'angx' or arg == 'angy' or arg == 'heading':
            return self.board.attitude[arg]
        else:
            print("Invalid argument\n")
            self.board.closeSerial()

    def signalControlExample(self):
        '''
        Sets the Roll/Pitch/Yaw on the Naze32 flight controller
        to maximum then minimum pulse widths
        '''
        self.reset()
        time.sleep(1)

        self.setYaw(self.MAX_WIDTH)
        self.setPitch(self.MAX_WIDTH)
        self.setRoll(self.MAX_WIDTH)

        time.sleep(2)

        self.setYaw(self.MIN_WIDTH)
        self.setPitch(self.MIN_WIDTH)
        self.setRoll(self.MIN_WIDTH)

        time.sleep(2)
        self.reset()
