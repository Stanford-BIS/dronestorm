"""Module to handle drone communication

Receives telemetry data: angx, angy and heading
Transmits attitude commands: roll, pitch, yaw
"""

from __future__ import division
from __future__ import print_function
import Adafruit_PCA9685

class DroneComm(object):
    """Handles communication to and from the drone.

    Receives data over the USB with Multiwii protocol
    Transmits data via an I2C interface to an Adafruit PWM generator

    Paramters
    ---------
    pwm_ctrl: bool
        enable pwm control (default True)
    period: float
        pwm period (default 22ms)
    k_period: float
        pwm period calibration factor
    roll_pwm_trim: int
        roll channel trim (us)
    pitch_pwm_trim: int
        pitch channel trim (us)
    yaw_pwm_trim: int
        yaw channel trim (us)
    port: string
        usb port attached to flight control board (default "/dev/ttyUSB0")
        if None, assumes no input connection from flight control board
    """
    # Range of Values (Pulse Width): 1.1ms -> 1.9ms
    MIN_WIDTH = 0.0011
    MID_WIDTH = 0.0015
    MAX_WIDTH = 0.0019

    # Max change in pulse width from mid posision: 0.4ms
    MAX_DELTA_PWIDTH = 0.0004

    # time precision of feather pwm signal
    # each pwm cycle is divided into TICKS units of time
    TICKS = 4096

    # Featherboard channel map
    ROLL_CHANNEL = 3
    PITCH_CHANNEL = 2
    YAW_CHANNEL = 1
    THR_CHANNEL = 0

    # Calibration factor to compensate for mismatch between
    # requested pwm period and pwm freq implemented by Adafruit PWM generator
    # Useage:
    #     requested_period = K_PWM * target_period
    # when requesting a PWM signal with period target_period
    DEFAULT_K_PERIOD = 0.023 / 0.022 # 23ms/22ms

    def __init__(
            self, pwm_ctrl=True,
            period=0.022, k_period=None,
            roll_trim=0, pitch_trim=0, yaw_trim=0,
            port="/dev/ttyUSB0"):
        self.period = period

        # store trims in units of seconds
        self.trim = {
            'roll':roll_trim * 1E-6,
            'pitch':pitch_trim * 1E-6,
            'yaw':yaw_trim * 1E-6
        }

        self.attitude = {'roll':0, 'ptich':0, 'yaw':0}
        self.imu = {
            'ax':0, 'ay':0, 'az':0,
            'droll':0, 'dpitch':0, 'dyaw':0,
            'mx':0, 'my':0, 'mz':0}

        if k_period is None:
            k_period = self.DEFAULT_K_PERIOD

        if pwm_ctrl:
            self.pwm = Adafruit_PCA9685.PCA9685()
            self.pwm.set_pwm_freq(1./(k_period*period))
            self.reset_channels()
        else:
            self.pwm = None

        if port is None:
            self.board = None
        else:
            self.board = MultiWii(port)

    def reset_channels(self):
        """Reset channels 0-6 on the feather board

        Applies trim to roll/pitch/yaw channels
        """
        for i in range(6):
            self.set_pwidth(i, self.MID_WIDTH)
        self.set_pwidth(
            self.ROLL_CHANNEL, self.MID_WIDTH + self.trim['roll'])
        self.set_pwidth(
            self.PITCH_CHANNEL, self.MID_WIDTH + self.trim['pitch'])
        self.set_pwidth(
            self.YAW_CHANNEL, self.MID_WIDTH + self.trim['yaw'])

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

    def set_roll_pwidth(self, width):
        """Apply trim and set the pwm Roll signal's positive pulse width

        Parameters
        ----------
        width: float
            positive pulse width (seconds)
        """
        # apply trim offset
        width += self.trim['roll']
        width, valid = self.validate_pwidth(width)
        self.set_pwidth(self.ROLL_CHANNEL, width)
        if not valid:
            print("WARNING: Requested roll pwidth out of range!")

    def set_pitch_pwidth(self, width):
        """Apply trim and set the pwm Pitch signal's positive pulse width

        Parameters
        ----------
        width: float
            positive pulse width (seconds)
        """
        # apply trim offset
        width += self.trim['pitch']
        width, valid = self.validate_pwidth(width)
        self.set_pwidth(self.PITCH_CHANNEL, width)
        if not valid:
            print("WARNING: Requested pitch pwidth out of range!")

    def set_yaw_pwidth(self, width):
        """Apply trim and set the pwm Yaw signal's positive pulse width

        Parameters
        ----------
        width: float
            positive pulse width (seconds)
        """
        # apply trim offset
        width += self.trim['yaw']
        width, valid = self.validate_pwidth(width)
        self.set_pwidth(self.YAW_CHANNEL, width)
        if not valid:
            print("WARNING: Requested yaw pwidth out of range!")

    def set_thr_pwidth(self, width):
        """Apply trim and set the pwm Throttle signal's positive pulse width

        Parameters
        ----------
        width: float
            positive pulse width (seconds)
        """
        # apply trim offset
        valid = True
        if width > self.MAX_WIDTH:
            valid = False
            width = self.MAX_WIDTH

        self.set_pwidth(self.THR_CHANNEL, width)
        if not valid:
            print("WARNING: Requested thr pwidth out of range!")

    def validate_pwidth(self, width):
        """Validate the pwm signal's positive pulse width

        Checks that the width is within the accepted range
        If not, the MAX_WIDTH or MIN_WIDTH is returned
        """
        if width > self.MAX_WIDTH:
            return self.MAX_WIDTH, False
        elif width < self.MIN_WIDTH:
            return self.MIN_WIDTH, False
        else:
            return width, True

    def set_roll_rate(self, rate):
        """Set the Roll rate

        Parameters
        ----------
        width: float [-1, 1]
            rate (scaled by max rate)
        """
        rate, valid = self.validate_rate(rate)
        width = (
            self.MID_WIDTH + self.trim['roll'] + rate*self.MAX_DELTA_PWIDTH)
        self.set_pwidth(self.ROLL_CHANNEL, width)
        if not valid:
            print("WARNING: Requested roll rate out of range!")

    def set_pitch_rate(self, rate):
        """Set the Pitch rate

        Parameters
        ----------
        width: float [-1, 1]
            rate (scaled by max rate)
        """
        rate, valid = self.validate_rate(rate)
        width = (
            self.MID_WIDTH + self.trim['pitch'] + rate*self.MAX_DELTA_PWIDTH)
        self.set_pwidth(self.PITCH_CHANNEL, width)
        if not valid:
            print("WARNING: Requested pitch rate out of range!")

    def set_yaw_rate(self, rate):
        """Set the Yaw rate

        Parameters
        ----------
        width: float [-1, 1]
            rate (scaled by max rate)
        """
        rate, valid = self.validate_rate(rate)
        width = (
            self.MID_WIDTH + self.trim['yaw'] + rate*self.MAX_DELTA_PWIDTH)
        self.set_pwidth(self.YAW_CHANNEL, width)
        if not valid:
            print("WARNING: Requested yaw rate out of range!")

    @staticmethod
    def validate_rate(rate):
        """Validate the requested channel rate is valid

        Checks that the width is within [-1, 1]
        Clips to range limit
        """
        if rate > 1:
            return 1, False
        elif rate < -1:
            return -1, False
        else:
            return rate, True

    def update_attitude(self):
        """Update the attitude data from the flight controller"""
        data = self.board.get_attitude()
        self.attitude['roll'] = data[0]
        self.attitude['pitch'] = data[1]
        self.attitude['yaw'] = data[2]

    def update_imu(self):
        """Updates the IMU data from the flight controller"""
        data = self.board.get_raw_imu()
        self.imu['ax'] = data[0]
        self.imu['ay'] = data[1]
        self.imu['az'] = data[2]
        self.imu['droll'] = data[3]
        self.imu['dpitch'] = data[4]
        self.imu['dyaw'] = data[5]
        self.imu['mx'] = data[6]
        self.imu['my'] = data[7]
        self.imu['mz'] = data[8]

    def exit(self):
        """
        Used to gracefully exit and close the serial port
        """
        if self.pwm is not None:
            self.reset_channels()
        if self.board is not None:
            self.board.close_serial()
