"""Forward input controls to output controls

run from terminal with
`python run_control_none.py`
"""
from __future__ import print_function
import os
from dronestorm.comm.redis_util import DBRedis, REDIS_RX_CHANNEL
import dronestorm.comm.redis_util as redis_util
from dronestorm.comm.rx_util import clip_rx
from dronestorm.print_util import print_control_header, print_control_data

def run_control_none():
    """Function to forward the receiver signals to the control signals

    Reads receiver data from redis database
    sets command data to receiver data clipped to [-1, 1]
    Writes command data to redis database
    """
    print(os.path.basename(__file__))
    db_redis = DBRedis()
    db_sub = db_redis.subscribe([REDIS_RX_CHANNEL])

    print("Running control_none...Ctrl-c to stop")
    print_control_header()
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
                print_control_data(rx, cmd)
    except KeyboardInterrupt:
        print("\nInterrupt received: stopping control...")

if __name__ == "__main__":
    run_control_none()
