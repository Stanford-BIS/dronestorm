"""This module communicates with the receiver during run time

Reads rc data from redis database
Writes data to the flight control board

run from terminal with
`python transmit_rc.py`
"""
from __future__ import print_function
import sys
from dronestorm.comm import MultiWii
from dronestorm.comm import msp
from dronestorm.redis_util import DBRedis, REDIS_RX_CHANNEL

def transmit_rc():
    """Function to forward rc commands to the flight control board"""
    db_redis = DBRedis()
    mw_comm = MultiWii()
    rx_sub = db_redis.subscribe(REDIS_RX_CHANNEL)
    print("Throttle     Roll    Pitch      Yaw     AUX1     AUX2")
    try:
        while True:
            # wait for data to be available
            rx_notice = rx_sub.get_message(timeout=10)
            if rx_notice is not None:
                rc_data = db_redis.get_rx()
                msp.set_rc(mw_comm, rc_data)
                sys.stdout.write(
                    "    %4d     %4d     %4d     %4d     %4d     %4d\r"%
                    tuple(rc_data[:6]))
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("Interrupt received: closing port and resetting settings...")
        mw_comm.close_serial()

if __name__ == "__main__":
    transmit_rc()
