"""Forward input controls to output controls

run from terminal with
`python run_control_none.py`
"""
from __future__ import print_function
import sys
import numpy as np
from dronestorm.redis_util import DBRedis, REDIS_RX_CHANNEL
import dronestorm.redis_util as redis_util
from dronestorm.comm.rx_util import rx_to_rx_rc, clip_rx
from dronestorm.comm.rx_util import (
    REMOTE_RX_THROTTLE_IDX,
    REMOTE_RX_DROLL_IDX, REMOTE_RX_DPITCH_IDX, REMOTE_RX_DYAW_IDX,
    REMOTE_RX_AUX1_IDX, REMOTE_RX_AUX2_IDX)
from dronestorm.control import PDController
from dronestorm.control.utils import find_min_angle

def print_control_none_header():
    """Utility to print header for control none"""
    print("                   RX                       | " +
        "                   Command ")
    print("Throttle  dRoll dPitch   dYaw " +
        "  AUX1   AUX2 | " +
        "Throttle  dRoll dPitch   dYaw " +
        "  AUX1   AUX2")

def print_control_none_data(rx, cmd):
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

def run_control_none():
    """Function to forward the control signals
    
    Reads receiver data from redis database
    sets command data to receiver data clipped to [-1, 1]
    Writes command data to redis database
    """
    db_redis = DBRedis()
    db_sub = db_redis.subscribe([REDIS_RX_CHANNEL])

    print("Running control_none...Ctrl-c to stop")
    print_control_none_header()
    try:
        while True:
            # check for new receiver or attitude data
            db_notice = db_sub.get_message(timeout=10)
            if db_notice is not None:
                # get receiver command as ref
                rx = redis_util.get_rx(db_redis)

                # forward most of rx data to cmd except yaw
                cmd = rx[:]
                clip_rx(cmd)

                redis_util.set_cmd(db_redis, cmd)
                print_control_none_data(rx, cmd)
    except KeyboardInterrupt:
        print("\nInterrupt received: stopping control...")

if __name__ == "__main__":
    run_control_none()
