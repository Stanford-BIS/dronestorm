"""This module simply forwards redis rx data to the flight control board

Reads rc data from redis database
Writes data to the flight control board

run from terminal with
`python forward_rx.py`
"""
from __future__ import print_function
from dronestorm.comm import MultiWii
from dronestorm.comm import msp
from dronestorm.log_util import print_rx_rx_rc_header, print_rx_rx_rc_data
from dronestorm.redis_util import DBRedis, REDIS_RX_CHANNEL
import dronestorm.redis_util as redis_util

def forward_rx():
    """Function to forward rc commands to the flight control board"""
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
    forward_rx()
