"""This module communicates with the receiver during run time

Reads data from the serial port attached to the receiver
Writes data to the redis database and notifies subscribers of new data

run from terminal with
`python receive_rx.py`
"""
from __future__ import print_function
import sys
from dronestorm.comm import SpektrumRemoteReceiver
from dronestorm.comm.rc_util import rx_rc_to_rx
from dronestorm.redis_util import DBRedis

def receive_rx():
    """Function to handles receiver communications"""
    db_redis = DBRedis()
    rx_comm = SpektrumRemoteReceiver()
    try:
        print("Waiting for receiver signal alignment...")
        rx_comm.align_serial()
        print("Receiver signal aligned. Receiving signal...Ctrl-c to stop")
        print("Throttle     Roll    Pitch      Yaw     AUX1     AUX2")
        while True:
            rx_rc_data = rx_comm.read_data()[:6] # only take first 6 channels
            rx_data = rx_rc_to_rx(rx_rc_data)
            db_redis.set_rx_rc(rx_rc_data)
            db_redis.set_rx(rx_data)
            sys.stdout.write(
                "%5.3f(%4d) %5.3f(%4d) %5.3f(%4d) %5.3f(%4d) %5.3f(%4d) %5.3f(%4d)\r"%
                tuple(rx_data))
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nInterrupt received: closing port and resetting settings...")
        rx_comm.close_serial()

if __name__ == "__main__":
    receive_rx()
