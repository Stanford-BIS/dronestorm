"""Forward redis receiver data to the flight control board during runtime

Reads rc data from redis database
Writes data to the flight control board

run from terminal with
`python run_drone_comm_foward_rx_rc.py`
"""
from __future__ import print_function
import os
import sys
from dronestorm.comm import MultiWii
from dronestorm.comm import msp
from dronestorm.comm.rx_util import (
    REMOTE_RX_THROTTLE_IDX,
    REMOTE_RX_DROLL_IDX, REMOTE_RX_DPITCH_IDX, REMOTE_RX_DYAW_IDX,
    REMOTE_RX_AUX1_IDX, REMOTE_RX_AUX2_IDX)
from dronestorm.redis_util import DBRedis, REDIS_RX_CHANNEL
import dronestorm.redis_util as redis_util

def print_rx_rx_rc_header():
    """Utility to print the rx and rx_rc header"""
    print("Throttle     Roll         Pitch        Yaw          " + 
          "AUX1         AUX2")

def print_rx_rx_rc_data(rx_data, rx_rc_data):
    """Utility to write rx and rx_rc data to stdout"""
    sys.stdout.write(
        "%+6.3f(%4d) "%(rx_data[REMOTE_RX_THROTTLE_IDX], rx_rc_data[REMOTE_RX_THROTTLE_IDX]) +
        "%+6.3f(%4d) "%(rx_data[REMOTE_RX_DROLL_IDX], rx_rc_data[REMOTE_RX_DROLL_IDX]) +
        "%+6.3f(%4d) "%(rx_data[REMOTE_RX_DPITCH_IDX], rx_rc_data[REMOTE_RX_DPITCH_IDX]) +
        "%+6.3f(%4d) "%(rx_data[REMOTE_RX_DYAW_IDX], rx_rc_data[REMOTE_RX_DYAW_IDX]) +
        "%+6.3f(%4d) "%(rx_data[REMOTE_RX_AUX1_IDX], rx_rc_data[REMOTE_RX_AUX1_IDX]) +
        "%+6.3f(%4d)"%(rx_data[REMOTE_RX_AUX2_IDX], rx_rc_data[REMOTE_RX_AUX2_IDX]) +
        "\r")
    sys.stdout.flush()

def run_drone_comm_forward_rx_rc():
    """Function to forward rc commands to the flight control board"""
    print(os.path.basename(__file__))
    db_redis = DBRedis()
    mw_comm = MultiWii()
    rx_sub = db_redis.subscribe(REDIS_RX_CHANNEL)
    print_rx_rx_rc_header()
    try:
        while True:
            # wait for data to be available
            rx_notice = rx_sub.get_message(timeout=10)
            if rx_notice is not None:
                rx_data = redis_util.get_rx(db_redis)
                rx_rc_data = redis_util.get_rx_rc(db_redis)
                msp.set_rc(mw_comm, rx_rc_data)
                print_rx_rx_rc_data(rx_data, rx_rc_data)
    except KeyboardInterrupt:
        print("Interrupt received: closing port and resetting settings...")
        mw_comm.close_serial()

if __name__ == "__main__":
    run_drone_comm_forward_rx_rc()
