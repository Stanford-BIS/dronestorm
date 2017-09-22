"""Compute yaw controlling outputs

run from terminal with
`python run_control_yaw.py`
"""
from __future__ import print_function
import sys
import numpy as np
from dronestorm.redis_util import (
        DBRedis, REDIS_RX_CHANNEL, REDIS_ATTITUDE_CHANNEL)
import dronestorm.redis_util as redis_util
from dronestorm.comm.rx_util import rx_to_rx_rc, clip_rx
from dronestorm.control import PDController
from dronestorm.control.utils import find_min_angle

def print_control_yaw_header():
    """Utility to print header for yaw control"""
    print("Attitude |    IMU |  "+
        "                   RX                      | " +
        "                   Command ")
    print("     Yaw |   dYaw | " +
        "Throttle  dRoll dPitch   dYaw " +
        "  AUX1   AUX2 | " +
        "Throttle  dRoll dPitch   dYaw " +
        "  AUX1   AUX2")

def print_control_yaw_data(yaw, dyaw, rx, cmd):
    """Utility to print yaw control data"""
    sys.stdout.write("  %+6.1f | "%(yaw))
    sys.stdout.write("%+6.f | "%(dyaw))
    sys.stdout.write(
        "  %+6.3f "%(rx[0]) +
        "%+6.3f "%(rx[1]) +
        "%+6.3f "%(rx[2]) +
        "%+6.3f "%(rx[3]) +
        "%+6.3f "%(rx[4]) +
        "%+6.3f |"%(rx[5]))
    sys.stdout.write(
        "   %+6.3f "%(cmd[0]) +
        "%+6.3f "%(cmd[1]) +
        "%+6.3f "%(cmd[2]) +
        "%+6.3f "%(cmd[3]) +
        "%+6.3f "%(cmd[4]) +
        "%+6.3f "%(cmd[5]) +
        "\r")
    sys.stdout.flush()

def run_control_yaw():
    """Function to compute the control signals for yaw control
    
    Reads attitude data from redis database
    Reads receiver data from redis database
    Writes command data to redis database
    """
    db_redis = DBRedis()
    db_sub_attitude = db_redis.subscribe(REDIS_ATTITUDE_CHANNEL)
    # wait for initial yaw data to use as reference point
    print("Initializing control_yaw. Waiting for initial yaw reading...")
    db_notice = db_sub_attitude.get_message(timeout=10000)
    yaw0 = redis_util.get_attitude(db_redis)[2]
    print("Initial yaw reading taken...")

    db_sub = db_redis.subscribe([REDIS_RX_CHANNEL, REDIS_ATTITUDE_CHANNEL])

    kp_yaw  = 0.009
    kd_yaw  = 0.00009
    yaw_controller = PDController(
        kp_yaw, kd_yaw, ref0=yaw0, center_error=find_min_angle, out_limit=1.)
    print("Running control_yaw...Ctrl-c to stop")
    print_control_yaw_header()
    try:
        while True:
            # check for new receiver or attitude data
            db_notice = db_sub.get_message(timeout=10)
            if db_notice is not None:
                # get state and dstate
                yaw = redis_util.get_attitude(db_redis)[2]
                dyaw = redis_util.get_imu(db_redis)[2]

                # get receiver command as ref
                rx = redis_util.get_rx(db_redis)
                ref_yaw = rx[3] * 180 # scale to -180 to 180

                cmd_yaw = yaw_controller.step(
                    yaw, dyaw, ref_yaw, dref=0.)

                # forward most of rx data to cmd except yaw
                cmd = rx[:]
                cmd[3] = cmd_yaw
                clip_rx(cmd)

                redis_util.set_cmd(db_redis, cmd)
                print_control_yaw_data(yaw, dyaw, rx, cmd)
    except KeyboardInterrupt:
        print("\nInterrupt received: stopping control...")

if __name__ == "__main__":
    run_control_yaw()
