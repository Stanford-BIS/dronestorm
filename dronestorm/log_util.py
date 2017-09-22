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
