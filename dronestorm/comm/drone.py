"""Module to handle drone communication

Receives telemetry data: angx, angy and heading
Transmits attitude commands: roll, pitch, yaw
"""
from __future__ import division
from __future__ import print_function
import numpy as np
from . import msp
from .rc_util import (
    RC_MIN, RC_MID, RC_MAX,
    range_1_to_rc, range_2_to_rc)

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
        self.rc_data = [RC_MID]*3 + [RC_MIN]*3
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

###############################################################################
# setters in RC units #########################################################
###############################################################################
def set_roll_rate_rc(drone_comm, roll_rate):
    """Apply trim and set the roll rate in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    roll_rate: int [1000, 2000]
        RC units of roll rate.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc_data[0] = np.clip(
        roll_rate + drone_comm.trim['roll'], RC_MIN, RC_MAX)

def set_pitch_rate_rc(drone_comm, pitch_rate):
    """Apply trim and set the pitch rate in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    pitch_rate: int [1000, 2000]
        RC units of pitch rate.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc_data[1] = np.clip(
        pitch_rate + drone_comm.trim['pitch'], RC_MIN, RC_MAX)

def set_yaw_rate_rc(drone_comm, yaw_rate):
    """Apply trim and set the yaw rate in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    yaw_rate: int [1000, 2000]
        RC units of yaw rate.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc_data[2] = np.clip(
        yaw_rate + drone_comm.trim['yaw'], RC_MIN, RC_MAX)

def set_throttle_rc(drone_comm, throttle):
    """Set the throttle in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    throttle: int [1000, 2000]
        RC units of throttle.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc_data[3] = np.clip(
        throttle, RC_MIN, RC_MAX)

def set_aux1_rc(drone_comm, aux1):
    """Set the AUX1 channel in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    aux1: int [1000, 2000]
        RC units of AUX1.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc_data[4] = np.clip(
        aux1, RC_MIN, RC_MAX)

def set_aux2_rc(drone_comm, aux2):
    """Apply trim and set the yaw rate in RC units

    Inputs
    ------
    drone_comm: instance of DroneComm
    aux2: int [1000, 2000]
        RC units of AUX2.
        RC units historically PWM positive pulse width in microseconds
    """
    drone_comm.rc_data[5] = np.clip(
        aux2, RC_MIN, RC_MAX)

def set_signals_rc(drone_comm, signals_rc):
    """Set the roll rate, pitch rate, yaw rate, throttle, AUX1, AUX2 data

    Inputs
    ------
    signals_rc: list of ints in [1000, 2000] range
        [roll_rate, pitch_rate, yaw_rate, throttle, AUX1, AUX2]
    """
    assert len(signals_rc) == 6, "signals must contain 6 channels of data"
    set_roll_rate_rc(drone_comm, signals_rc[0])
    set_pitch_rate_rc(drone_comm, signals_rc[1])
    set_yaw_rate_rc(drone_comm, signals_rc[2])
    set_throttle_rc(drone_comm, signals_rc[3])
    set_aux1_rc(drone_comm, signals_rc[4])
    set_aux2_rc(drone_comm, signals_rc[5])

###############################################################################
# utilities ###################################################################
###############################################################################

def _validate_rate(rate, min_value=-1, max_value=1):
    """Validates that rate is within [min_value, max_value]

    Clips rates and returns whether rate was within range or not
    """
    if rate > max_value:
        return max_value, False
    elif rate < min_value:
        return min_value, False
    else:
        return rate, True

###############################################################################
# setters in normalized units #################################################
###############################################################################
def set_roll_rate(drone_comm, roll_rate):
    """Set the roll rate

    Parameters
    ----------
    drone_comm: instance of DroneComm
    roll_rate: float
        roll rate normalized to [-1, 1]
    """
    roll_rate, valid = _validate_rate(roll_rate)
    rc_value = range_2_to_rc(roll_rate)
    set_roll_rate_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested roll rate out of range!")

def set_pitch_rate(drone_comm, pitch_rate):
    """Set the pitch rate

    Parameters
    ----------
    drone_comm: instance of DroneComm
    pitch_rate: float
        pitch rate normalized to [-1, 1]
    """
    pitch_rate, valid = _validate_rate(pitch_rate)
    rc_value = range_2_to_rc(pitch_rate)
    set_pitch_rate_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested pitch rate out of range!")

def set_yaw_rate(drone_comm, yaw_rate):
    """Set the yaw rate

    Parameters
    ----------
    drone_comm: instance of DroneComm
    yaw_rate: float
        yaw rate normalized to [-1, 1]
    """
    yaw_rate, valid = _validate_rate(yaw_rate)
    rc_value = range_2_to_rc(yaw_rate)
    set_yaw_rate_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested yaw rate out of range!")

def set_throttle(drone_comm, throttle):
    """Set the throttle

    Parameters
    ----------
    drone_comm: instance of DroneComm
    throttle: float
        throttle normalized to [0, 1]
    """
    throttle, valid = _validate_rate(throttle, min_value=0)
    rc_value = range_1_to_rc(throttle)
    set_throttle_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested throttle out of range!")

def set_aux1(drone_comm, aux1):
    """Set the aux1

    Parameters
    ----------
    drone_comm: instance of DroneComm
    aux1: float
        AUX1 normalized to [0, 1]
    """
    aux1, valid = _validate_rate(aux1, min_value=0)
    rc_value = range_1_to_rc(aux1)
    set_aux1_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested AUX1 out of range!")

def set_aux2(drone_comm, aux2):
    """Set the aux2

    Parameters
    ----------
    drone_comm: instance of DroneComm
    aux2: float
        AUX2 normalized to [0, 1]
    """
    aux2, valid = _validate_rate(aux2, min_value=0)
    rc_value = range_1_to_rc(aux2)
    set_aux2_rc(drone_comm, rc_value)
    if not valid:
        print("WARNING: Requested AUX2 out of range!")

def set_signals(drone_comm, signals):
    """Set the roll rate, pitch rate, yaw rate, throttle, AUX1, AUX2 data

    Inputs
    ------
    channel_data: list of floats
        [roll_rate, pitch_rate, yaw_rate, throttle, AUX1, AUX2]
        roll_rate, pitch_rate, yaw_rate in [-1, 1] range
        throttle, AUX1, AUX2 in [0, 1] range
    """
    assert len(signals) == 6, "signals must contain 6 channels of data"
    set_roll_rate(drone_comm, signals[0])
    set_pitch_rate(drone_comm, signals[1])
    set_yaw_rate(drone_comm, signals[2])
    set_throttle(drone_comm, signals[3])
    set_aux1(drone_comm, signals[4])
    set_aux2(drone_comm, signals[5])

###############################################################################
# getters #####################################################################
###############################################################################
def get_attitude(drone_comm):
    """Retrieve the attitude data from a DroneComm instance

    Inputs
    ------
    drone_comm: DroneComm instance
    """
    roll = drone_comm.attitude["roll"]
    pitch = drone_comm.attitude["pitch"]
    yaw = drone_comm.attitude["yaw"]
    return list(roll, pitch, yaw)

def get_imu(drone_comm):
    """Retrieve the imu data from a DroneComm instance

    Inputs
    ------
    drone_comm: DroneComm instance
    """
    ax = drone_comm.imu["ax"]
    ay = drone_comm.imu["ay"]
    az = drone_comm.imu["az"]
    droll = drone_comm.imu["droll"]
    dpitch = drone_comm.imu["dpitch"]
    dyaw = drone_comm.imu["dyaw"]
    return [ax, ay, az, droll, dpitch, dyaw]
