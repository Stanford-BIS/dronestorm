"""
Library to communicate with the Naze32 flight control board.
Receives telemetry data: angx, angy and heading
Transmits attitude commands: roll, pitch, yaw
"""

from __future__ import division
from pyMultiwii import MultiWii
import Adafruit_PCA9685, time

class DroneComm(object):
    """Handles communication to and from the drone.

    Receives data over the USB with Multiwii protocol
    Transmits data via an I2C interface to an Adafruit PWM generator
    """
    # Range of Values (Pulse Width): 1.1ms -> 1.9ms
    MIN_WIDTH = 1.10
    MAX_WIDTH = 1.90
    MED_WIDTH = 1.50

    # Featherboard channels
    YAW_CHANNEL = 1
    PITCH_CHANNEL = 2
    ROLL_CHANNEL = 3

    # Calibration factor to compensate for mismatch between requested pwm width
    # and width implemented by Adafruit PWM generator
    # Use:
    #     requested_p_signal = K_PWM * target_p_width
    # when requesting a PWM signal with positive witdth target_p_width 
    K_PWM = 1.4 / 1.5

    def __init__(self):
        """Initializes PWM and MultiWii objects"""
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(1/.023)    # ~45.45 Hz
        self.board = MultiWii("/dev/ttyUSB0")
        self.reset_channels() # Used to activate the Featherboard

    def set_servo_pulse(self, channel, pulse):
        """Set a certain Pulse Width on a channel"""
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 1/.022   # ~45.45 Hz
        pulse_length //= 4096     # 12 bits of resolution
        pulse *= 1000
        pulse //= pulse_length

        self.pwm.set_pwm(channel, 0, int(pulse))

    def convert_width(self, width):
        """Scales desired pusle width before request to Adafruit library"""
        return width * self.K_PWM

    def set_yaw(self, width):
        """Sets the Yaw to a desired PWM width"""
        width_c, valid = self.is_valid(width)
        self.set_servo_pulse(self.YAW_CHANNEL, width_c)

        if not valid:
            print("WARNING: Yaw out of range!")

    def set_pitch(self, width):
        '''
        Sets the Pitch to a desired PWM width
        '''
        width_c, valid = self.is_valid(width)
        self.set_servo_pulse(self.PITCH_CHANNEL, width_c)

        if not valid:
            print("WARNING: Pitch out of range!")

    def set_roll(self, width):
        '''
        Sets the Roll to a desired PWM width
        '''
        width_c, valid = self.is_valid(width)
        self.set_servo_pulse(self.ROLL_CHANNEL, width_c)

        if not valid:
            print("WARNING: Roll out of range!")

    def is_valid(self, width):
        '''
        Verifies that the PWM signals are in the accepted range.
        If not, the MAX_WIDTH or MIN_WIDTH is returned
        '''
        if(width > self.MAX_WIDTH):
            return self.convert_width(self.MAX_WIDTH), False
        elif(width < self.MIN_WIDTH):
            return self.convert_width(self.MIN_WIDTH), False
        else:
            return self.convert_width(width), True

    def reset_channels(self):
        '''
        Resets channels 0-6 on the feather board; used as cleanup measure
        '''
        for i in range(6):
            self.set_servo_pulse(i, self.convert_width(self.MED_WIDTH))

    def get_data(self, arg):
        '''
        Returns the Attitude telemetry data from the Naze32 flight controller

        param: MultiWii Object, {"angx", "angy", "heading"}
        return: {angx, angy, heading}
        '''
        self.board.getData(MultiWii.ATTITUDE)

        if arg == 'angx' or arg == 'angy' or arg == 'heading':
            return self.board.attitude[arg]
        else:
            print("Invalid argument\n")
            self.board.closeSerial()

    def get_roll(self):
        """Returns the roll angle"""
        self.board.getData(MultiWii.ATTITUDE)
        return self.board.attitude["angy"]

    def get_pitch(self):
        """Returns the pitch angle"""
        self.board.getData(MultiWii.ATTITUDE)
        return self.board.attitude["angx"]

    def get_yaw(self):
        """Returns the yaw angle"""
        self.board.getData(MultiWii.ATTITUDE)
        return self.board.attitude["heading"]

    def exit(self):
        '''
        Used to gracefully exit and close the serial port
        '''
        self.reset_channels()
        self.board = MultiWii("/dev/ttyUSB0")
        self.board.closeSerial()

    def control_example(self):
        '''
        Sets the Roll/Pitch/Yaw on the Naze32 flight controller
        to maximum then minimum pulse widths
        '''
        self.reset_channels()
        time.sleep(1)

        self.set_yaw(self.MAX_WIDTH)
        self.set_pitch(self.MAX_WIDTH)
        self.set_roll(self.MAX_WIDTH)

        time.sleep(2)

        self.set_yaw(self.MIN_WIDTH)
        self.set_pitch(self.MIN_WIDTH)
        self.set_roll(self.MIN_WIDTH)

        time.sleep(2)
        self.reset_channels()
