"""communicates with the receiver during runtime

Reads data from the serial port attached to the receiver
Writes data to the redis database and notifies subscribers of new data

run from terminal with
`python run_receiver.py`
"""
from __future__ import print_function
from dronestorm.comm import SpektrumRemoteReceiver
from dronestorm.comm.rc_util import rx_rc_to_rx
from dronestorm.log_util import print_rx_rx_rc_header, print_rx_rx_rc_data
from dronestorm.redis_util import DBRedis
import dronestorm.redis_util as redis_util

def run_receiver():
    """Function to handles receiver communications"""
    db_redis = DBRedis()
    rx_comm = SpektrumRemoteReceiver()
    try:
        print("Waiting for receiver signal alignment...")
        rx_comm.align_serial()
        print("Receiver signal aligned. Receiving signal...Ctrl-c to stop")
        print_rx_rx_rc_header()
        while True:
            rx_rc_data = rx_comm.read_data()[:6] # only take first 6 channels
            rx_data = rx_rc_to_rx(rx_rc_data)
            redis_util.set_rx_rc(db_redis, rx_rc_data)
            redis_util.set_rx(db_redis, rx_data)
            print_rx_rx_rc_data(rx_data, rx_rc_data)
    except KeyboardInterrupt:
        print("\nInterrupt received: closing port and resetting settings...")
        rx_comm.close_serial()

if __name__ == "__main__":
    run_receiver()
