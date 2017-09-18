"""Module for receiver constants and utility functions

Radio control values are historically tied to PWM values
Specifically, the positive pulse width in microseconds: 1000us to 2000us
PWM signals were used to directly drive servos and motors
Nowadaws, we communicate over digital protocols, but lots of firmware
out in the wild still use values in the 1000-2000 range
"""
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

def range_1_to_rc(range_1_signal):
    """Convert signals normalized to [0, 1] to rc values of [1000, 2000]""" 
    return RC_MIN + range_1_signal*RC_RANGE

def range_2_to_rc(range_2_signal):
    """Convert signals normalized to [-1, 1] to rc values of [1000, 2000]""" 
    return RC_MID + range_2_signal*RC_HALF_RANGE

def rc_to_range_1(rc_signal):
    """Normalize rc values in range [1000, 2000] to range [0, 1]""" 
    return (rc_signal - RC_MIN) * RC_RANGE_INV

def rc_to_range_2(rc_signal):
    """Normalize rc values in range [1000, 2000] to range [-1, 1]""" 
    return (rc_signal - RC_MID) * RC_HALF_RANGE_INV

def rx_rc_to_rx(rx_rc):
    """Convert list of rx data in rc units to [0, 1], [-1, 1]
    
    Inputs
    ------
    rx_rc: list of rx data in RC units
        [roll_rate, pitch_rate, yaw_rate, throttle, AUX1, AUX2]

    Returns a list of rx data in normalized units
    """
    rx_normalized = ([rc_to_range_1(rx_rc[0])] +
                     list(map(rc_to_range_2, rx_rc[1:4])) + 
                     list(map(rc_to_range_1, rx_rc[4:])))
    return rx_normalized
