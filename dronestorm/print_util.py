"""Utility module for printing"""
import sys
from dronestorm.comm.rx_util import (
    REMOTE_RX_THROTTLE_IDX,
    REMOTE_RX_DROLL_IDX, REMOTE_RX_DPITCH_IDX, REMOTE_RX_DYAW_IDX,
    REMOTE_RX_AUX1_IDX, REMOTE_RX_AUX2_IDX)

def print_control_header():
    """Utility to print header for control modules"""
    print("                   RX                       | " +
          "                   Command ")
    print("Throttle  dRoll dPitch   dYaw " +
          "  AUX1   AUX2 | " +
          "Throttle  dRoll dPitch   dYaw " +
          "  AUX1   AUX2")

def print_control_data(rx, cmd):
    """Utility to print rx and control data"""
    sys.stdout.write(
        "  %+6.3f "%(rx[REMOTE_RX_THROTTLE_IDX]) +
        "%+6.3f "%(rx[REMOTE_RX_DROLL_IDX]) +
        "%+6.3f "%(rx[REMOTE_RX_DPITCH_IDX]) +
        "%+6.3f "%(rx[REMOTE_RX_DYAW_IDX]) +
        "%+6.3f "%(rx[REMOTE_RX_AUX1_IDX]) +
        "%+6.3f |"%(rx[REMOTE_RX_AUX2_IDX]))
    sys.stdout.write(
        "   %+6.3f "%(cmd[REMOTE_RX_THROTTLE_IDX]) +
        "%+6.3f "%(cmd[REMOTE_RX_DROLL_IDX]) +
        "%+6.3f "%(cmd[REMOTE_RX_DPITCH_IDX]) +
        "%+6.3f "%(cmd[REMOTE_RX_DYAW_IDX]) +
        "%+6.3f "%(cmd[REMOTE_RX_AUX1_IDX]) +
        "%+6.3f "%(cmd[REMOTE_RX_AUX2_IDX]) +
        "\r")
    sys.stdout.flush()
