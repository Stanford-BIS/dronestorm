"""This module handles communication with the receiver during run time

run from terminal with
`python receive_rx.py`
"""
from dronestorm.comm import SpektrumRemoteReceiver
from dronestorm.redis_util import DBRedis

def run_receiver():
    """Function to handles receiver communications"""
    db_redis = DBRedis()
    rx = SpektrumRemoteReceiver()
    rx.align_serial()
    try:
        while True:
            rx_data = rx.read_data()
            db_redis.set_imu(rx_data)
    except KeyboardInterrupt:
        print("Interrupt received: closing port and resetting settings...")
        rx.close_serial()

if __name__ == "__main__":
    run_receiver()
