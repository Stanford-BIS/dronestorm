"""Module to handle drone communication

Receives telemetry data: angx, angy and heading
Transmits attitude commands: roll, pitch, yaw
"""
from __future__ import division
from __future__ import print_function
import numpy as np
from . import msp

# Range of Values (historically Pulse Width of PWM): 1000us to 2000us
MIN_RC_VALUE = 1000
MID_RC_VALUE = 1500
MAX_RC_VALUE = 2000
RC_RANGE = MAX_RC_VALUE - MIN_RC_VALUE
RC_HALF_RANGE = RC_RANGE/2

class DroneComm(object):
    """Handles communication to and from the flight control board.

    Transmits and Receives data over the USB with Multiwii protocol

    Paramters
    ---------
    roll_trim: int
        roll channel trim (us)
    pitch_rim: int
        pitch channel trim (us)
    yaw_trim: int
        yaw channel trim (us)
    port: string
        usb port attached to flight control board (default "/dev/ttyUSB0")
        if None, assumes no input connection from flight control board
    """
    def __init__(self, roll_trim=0, pitch_trim=0, yaw_trim=0,
                 port="/dev/ttyACM0"):
        self.trim = {
            'roll':roll_trim,
            'pitch':pitch_trim,
            'yaw':yaw_trim,
        }

        self.attitude = {'roll':0, 'ptich':0, 'yaw':0}
        self.imu = {
            'ax':0, 'ay':0, 'az':0,
            'droll':0, 'dpitch':0, 'dyaw':0,}

        # rc signals
        # roll, pitch, yaw, throttle, aux1, aux2
        self.rc_data = [MID_RC_VALUE]*3 + [MIN_RC_VALUE]*3
        self.mw_comm = msp.MultiWii(port)
        self.send_rc(self.rc_data)

    def send_rc(self, rc_data):
        """Send RC data to the flight control board"""
        msp.set_rc(self.mw_comm, rc_data)

    def update_attitude(self):
        """Update the attitude data from the flight controller"""
        data = msp.get_attitude(self.mw_comm)
        self.attitude['roll'] = data[0]
        self.attitude['pitch'] = data[1]
        self.attitude['yaw'] = data[2]

    def update_imu(self):
        """Updates the IMU data from the flight controller"""
        data = msp.get_raw_imu(self.mw_comm)
        self.imu['ax'] = data[0]
        self.imu['ay'] = data[1]
        self.imu['az'] = data[2]
        self.imu['droll'] = data[3]
        self.imu['dpitch'] = data[4]
        self.imu['dyaw'] = data[5]

    def exit(self):
        """
        Used to gracefully exit and close the serial port
        """
        self.mw_comm.close_serial()

# setters & getters in RC units ###############################################
def set_roll_rate_rc(drone_comm, roll_rate):
    """Apply trim and set the roll rate in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    roll_rate: int [1000, 2000]
        RC units of roll rate.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc[0] = np.clip(
        roll_rate + drone_comm.trim['roll'], MIN_RC_VALUE, MAX_RC_VALUE)

def set_pitch_rate_rc(drone_comm, pitch_rate):
    """Apply trim and set the pitch rate in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    pitch_rate: int [1000, 2000]
        RC units of pitch rate.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc[1] = np.clip(
        pitch_rate + drone_comm.trim['pitch'], MIN_RC_VALUE, MAX_RC_VALUE)

def set_yaw_rate_rc(drone_comm, yaw_rate):
    """Apply trim and set the yaw rate in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    yaw_rate: int [1000, 2000]
        RC units of yaw rate.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc[2] = np.clip(
        yaw_rate + drone_comm.trim['yaw'], MIN_RC_VALUE, MAX_RC_VALUE)

def set_throttle_rc(drone_comm, throttle):
    """Set the throttle in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    throttle: int [1000, 2000]
        RC units of throttle.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc[3] = np.clip(
        throttle, MIN_RC_VALUE, MAX_RC_VALUE)

def set_aux1_rc(drone_comm, aux1):
    """Set the AUX1 channel in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    aux1: int [1000, 2000]
        RC units of AUX1.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc[4] = np.clip(
        aux1, MIN_RC_VALUE, MAX_RC_VALUE)

def set_aux2_rc(drone_comm, aux2):
    """Apply trim and set the yaw rate in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    aux2: int [1000, 2000]
        RC units of AUX2.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc[5] = np.clip(
        aux2, MIN_RC_VALUE, MAX_RC_VALUE)

# setters & getters in [-1,1] units ###########################################
def validate_rate(rate, min_value=-1, max_value=1):
    """Validates that rate is within [min_value, max_value]

    Clips rates and returns whether rate was within range or not
    """
    if rate > max_value:
        return max_value, False
    elif rate < min_value:
        return min_value, False
    else:
        return rate, True

def set_roll_rate(drone_comm, rate):
    """Set the roll rate

    Parameters
    ----------
    drone_comm: instance of DroneComm
    rate: float
        roll rate normalized to [-1, 1]
    """
    rate, valid = validate_rate(rate)
    rc_value = MID_RC_VALUE + rate*RC_HALF_RANGE
    set_roll_rate_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested roll rate out of range!")

def set_pitch_rate(drone_comm, rate):
    """Set the pitch rate

    Parameters
    ----------
    drone_comm: instance of DroneComm
    rate: float
        pitch rate normalized to [-1, 1]
    """
    rate, valid = validate_rate(rate)
    rc_value = MID_RC_VALUE + rate*RC_HALF_RANGE
    set_pitch_rate_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested pitch rate out of range!")

def set_yaw_rate(drone_comm, rate):
    """Set the yaw rate

    Parameters
    ----------
    drone_comm: instance of DroneComm
    rate: float
        yaw rate normalized to [-1, 1]
    """
    rate, valid = validate_rate(rate)
    rc_value = MID_RC_VALUE + rate*RC_HALF_RANGE
    set_yaw_rate_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested yaw rate out of range!")

def set_throttle(drone_comm, rate):
    """Set the throttle

    Parameters
    ----------
    drone_comm: instance of DroneComm
    rate: float
        throttle normalized to [0, 1]
    """
    rate, valid = validate_rate(rate, min_value=0)
    rc_value = MIN_RC_VALUE + rate*RC_RANGE
    set_throttle_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested throttle out of range!")

def set_aux1(drone_comm, rate):
    """Set the aux1

    Parameters
    ----------
    drone_comm: instance of DroneComm
    rate: float
        AUX1 normalized to [0, 1]
    """
    rate, valid = validate_rate(rate, min_value=0)
    rc_value = MIN_RC_VALUE + rate*RC_RANGE
    set_aux1_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested AUX1 out of range!")

def set_aux2(drone_comm, rate):
    """Set the aux2

    Parameters
    ----------
    drone_comm: instance of DroneComm
    rate: float
        AUX2 normalized to [0, 1]
    """
    rate, valid = validate_rate(rate, min_value=0)
    rc_value = MIN_RC_VALUE + rate*RC_RANGE
    set_aux2_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested AUX2 out of range!")
