"""Utilities for logging"""
from __future__ import print_function
import sys

def print_rx_rx_rc_header():
    """Utility to print the rx and rx_rc header"""
    print("Throttle     Roll         Pitch        Yaw          " + 
          "AUX1         AUX2")

def print_rx_rx_rc_data(rx_data, rx_rc_data):
    """Utility to write rx and rx_rc data to stdout"""
    sys.stdout.write(
        "%+6.3f(%4d) "%(rx_data[0], rx_rc_data[0]) +
        "%+6.3f(%4d) "%(rx_data[1], rx_rc_data[1]) +
        "%+6.3f(%4d) "%(rx_data[2], rx_rc_data[2]) +
        "%+6.3f(%4d) "%(rx_data[3], rx_rc_data[3]) +
        "%+6.3f(%4d) "%(rx_data[4], rx_rc_data[4]) +
        "%+6.3f(%4d)"%(rx_data[5], rx_rc_data[5]) +
        "\r")
    sys.stdout.flush()

def print_drone_comm_header():
    """Utility to print the header for drone comm"""
    print(
        "      Attitude       | " +
        "                  IMU                  | " +
        "                                  Command ")
    print(
        "  Roll  Pitch    Yaw | " +
        "   ax    ay    az  dRoll dPitch   dYaw |" +
        "        dRoll      dPitch         dYaw    " +
        "  Throttle         AUX1         AUX2")

def print_drone_comm(attitude, imu, cmd, cmd_rc):
    """Utility to print drone comm data"""
    sys.stdout.write("%+6.1f %+6.1f %+6.1f | "%tuple(attitude))
    sys.stdout.write("%+5.f %+5.f %+5.f %+6.f %+6.f %+6.f | "%tuple(imu))
    sys.stdout.write(
        "%+6.3f(%4d) "%(cmd[0], cmd_rc[0]) +
        "%+6.3f(%4d) "%(cmd[1], cmd_rc[1]) +
        "%+6.3f(%4d) "%(cmd[2], cmd_rc[2]) +
        "%+6.3f(%4d) "%(cmd[3], cmd_rc[3]) +
        "%+6.3f(%4d) "%(cmd[4], cmd_rc[4]) +
        "%+6.3f(%4d) "%(cmd[5], cmd_rc[5]) +
        "\r")
    sys.stdout.flush()
