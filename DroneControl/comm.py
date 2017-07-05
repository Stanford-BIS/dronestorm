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

    Paramters
    ---------
    period: float
        pwm period (default 22ms)
    k_period: float
        pwm period calibration factor
    k_pwidth: float
        pwm positive pulse width calibration factor
    roll_pwm_trim:  int
        roll channel trim
    pitch_pwm_trim: int
        pitch channel trim
    yaw_pwm_trim:   int
        yaw channel trim
    port: string
        usb port attached to flight control board (default "/dev/ttyUSB0")
        if None, assumes no input connection from flight control board
    """
    # Range of Values (Pulse Width): 1.1ms -> 1.9ms
    MIN_WIDTH = 0.0011
    MID_WIDTH = 0.0015
    MAX_WIDTH = 0.0019

    # time precision of feather pwm signal
    # each pwm cycle is divided into TICKS units of time
    TICKS = 4096

    # Featherboard channels
    YAW_CHANNEL   = 1
    PITCH_CHANNEL = 2
    ROLL_CHANNEL  = 3

    # Calibration factor to compensate for mismatch between
    # requested pwm period and pwm freq implemented by Adafruit PWM generator
    # Useage:
    #     requested_period = K_PWM * target_period
    # when requesting a PWM signal with period target_period
    DEFAULT_K_PERIOD = 0.023 / 0.022 # 23ms/22ms

    def __init__(
            self, period=0.022, k_period=None, k_pwidth=None,
            roll_pwm_trim=0, pitch_pwm_trim=0, yaw_pwm_trim=0,
            port="/dev/ttyUSB0"):
        """Initializes PWM and MultiWii objects"""
        self.period = period

        if k_period is None:
            k_period = self.DEFAULT_K_PERIOD

        if k_pwidth is None:
            self.k_pwidth = self.DEFAULT_K_PWIDTH
        else:
            self.k_pwdith = k_pwidth

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(1./(k_period*period))
        if port is None:
            self.board = None
        else:
            self.board = MultiWii(port)

        self.reset_channels()

    def set_pwidth(self, channel, width):
        """Set a positive Pulse Width on a channel
        
        Parameters
        ----------
        channel: int
            pwm channel to set positive pulse width
        width: float
            positive pulse width (seconds)
        """
        width = width / self.period * self.TICKS
        pulse = int(round(width))
        self.pwm.set_pwm(channel, 0, pulse)

    def set_yaw_pwm(self, width):
        """Sets the Yaw to a desired PWM width"""
        # apply trim offset
        width_c, valid = self.is_valid(width)
        self.set_pwidth(self.YAW_CHANNEL, width_c)

        if not valid:
            print("WARNING: Yaw out of range!")

    def set_pitch_pwm(self, width):
        """Sets the Pitch to a desired PWM width"""
        # apply trim offset
        width_c, valid = self.is_valid(width)
        self.set_pwidth(self.PITCH_CHANNEL, width_c)

        if not valid:
            print("WARNING: Pitch out of range!")

    def set_roll_pwm(self, width):
        """Sets the Roll to a desired PWM width"""
        width_c, valid = self.is_valid(width)
        self.set_pwidth(self.ROLL_CHANNEL, width_c)

        if not valid:
            print("WARNING: Roll out of range!")

    def is_valid(self, width):
        """Verifies that the PWM signals are in the accepted range.

        If not, the MAX_WIDTH or MIN_WIDTH is returned
        """
        if(width > self.MAX_WIDTH):
            return self.convert_width(self.MAX_WIDTH), False
        elif(width < self.MIN_WIDTH):
            return self.convert_width(self.MIN_WIDTH), False
        else:
            return self.convert_width(width), True

    def reset_channels(self):
        """Reset channels 0-6 on the feather board; used as cleanup measure"""
        for i in range(6):
            self.set_pwidth(i, self.MID_WIDTH)

    def get_data(self, arg):
        """
        Returns the Attitude telemetry data from the Naze32 flight controller

        param: MultiWii Object, {"angx", "angy", "heading"}
        return: {angx, angy, heading}
        """
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
        """
        Used to gracefully exit and close the serial port
        """
        self.reset_channels()
        self.board = MultiWii("/dev/ttyUSB0")
        self.board.closeSerial()

    def control_example(self):
        """
        Sets the Roll/Pitch/Yaw on the Naze32 flight controller
        to maximum then minimum pulse widths
        """
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
