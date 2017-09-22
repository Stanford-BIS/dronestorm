"""Module for receiver constants and utility functions

Radio control values are historically tied to PWM values
Specifically, the positive pulse width in microseconds: 1000us to 2000us
PWM signals were used to directly drive servos and motors
Nowadaws, we communicate over digital protocols, but lots of firmware
out in the wild still use values in the 1000-2000 range
"""
import numpy as np

RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000
RC_RANGE = RC_MAX - RC_MIN
RC_HALF_RANGE = RC_RANGE/2
RC_RANGE_INV = 1./RC_RANGE
RC_HALF_RANGE_INV = 1./RC_HALF_RANGE

# Value range communicated by the receiver over remote serial protocol
RC_MIN_SERIAL = 988
RC_MAX_SERIAL = 2011

# Spektrum Remote Receiver protocol channel indexing
(REMOTE_RX_THROTTLE_IDX,
 REMOTE_RX_DROLL_IDX,
 REMOTE_RX_DPITCH_IDX,
 REMOTE_RX_DYAW_IDX,
 REMOTE_RX_AUX1_IDX,
 REMOTE_RX_AUX2_IDX) = [0, 1, 2, 3, 4, 5]

# MultiWii Serial Protocol channel indexing
(MSP_THROTTLE_IDX,
 MSP_DROLL_IDX,
 MSP_DPITCH_IDX,
 MSP_DYAW_IDX,
 MSP_AUX1_IDX,
 MSP_AUX2_IDX) = [0, 1, 2, 3, 4, 5]

def range_1_to_rc(range_1_signal):
    """Convert signals normalized to [0, 1] to rc values of [1000, 2000]""" 
    return int(round(RC_MIN + range_1_signal*RC_RANGE))

def range_2_to_rc(range_2_signal):
    """Convert signals normalized to [-1, 1] to rc values of [1000, 2000]""" 
    return int(round(RC_MID + range_2_signal*RC_HALF_RANGE))

def rc_to_range_1(rc_signal):
    """Normalize rc values in range [1000, 2000] to range [0, 1]""" 
    return (rc_signal - RC_MIN) * RC_RANGE_INV

def rc_to_range_2(rc_signal):
    """Normalize rc values in range [1000, 2000] to range [-1, 1]""" 
    return (rc_signal - RC_MID) * RC_HALF_RANGE_INV

def rx_rc_to_rx(rx_rc):
    """Convert list of rx data in rc units to [-1, 1], [0, 1] normalized ranges
    
    Inputs
    ------
    rx_rc: list of rx data in RC units
        [throttle, roll_rate, pitch_rate, yaw_rate, AUX1, AUX2]

    Returns a list of rx data in normalized units
    """
    rx_normalized = ([rc_to_range_1(rx_rc[0])] +
                     list(map(rc_to_range_2, rx_rc[1:4])) + 
                     list(map(rc_to_range_1, rx_rc[4:])))
    return rx_normalized

def rx_to_rx_rc(rx_normed):
    """Convert list of rx data in [-1, 1], [0, 1] normalized ranges to rc units
    
    Inputs
    ------
    rx: list of rx data in RC [-1, 1], [0, 1] 
        [throttle, roll_rate, pitch_rate, yaw_rate, AUX1, AUX2]

    Returns a list of rx data in rc units [1000, 2000]
    """
    rx_rc = ([range_1_to_rc(rx_normed[0])] +
                     list(map(range_2_to_rc, rx_normed[1:4])) + 
                     list(map(range_1_to_rc, rx_normed[4:])))
    return rx_rc

def clip_rx(rx_data):
    """Clips rx channels to [-1, 1] or [0,1] range
    
    Expects channels in Spektrum Remote Receiver Indexing order
        [throttle, roll_rate, pitch_rate, yaw_rate, AUX1, AUX2]
    """
    rx_data[0] = np.clip(rx_data[REMOTE_RX_THROTTLE_IDX], 0, 1)
    rx_data[1] = np.clip(rx_data[REMOTE_RX_DROLL_IDX], -1, 1)
    rx_data[2] = np.clip(rx_data[REMOTE_RX_DPITCH_IDX], -1, 1)
    rx_data[3] = np.clip(rx_data[REMOTE_RX_DYAW_IDX], -1, 1)
    rx_data[4] = np.clip(rx_data[REMOTE_RX_AUX1_IDX], 0, 1)
    rx_data[5] = np.clip(rx_data[REMOTE_RX_AUX2_IDX], 0, 1)
