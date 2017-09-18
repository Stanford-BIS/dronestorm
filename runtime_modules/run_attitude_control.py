"""Compute attitude controlling outputs during runtime

run from terminal with
`python run_attitude_control.py`
"""
from __future__ import print_function
from dronestorm.redis_util import DBRedis
import dronestorm.redis_util as redis_util
from dronestorm.control import AttitudePID

def run_attitude_control():
    """Function to compute the control signals for attitude control
    
    Reads IMU data to redis database
    Reads receiver data from redis database
    Writes command data to redis database
    """
    db_redis = DBRedis()
    db_sub = db_redis.subscribe([REDIS_RX_CHANNEL, REDIS_ATTITUDE_CHANNEL])
    attitude_controller = AttitudePID(...)
    try:
        print("Running attitude control...Ctrl-c to stop")
        while True:
            # check for new receiver or attitude data
            db_notice = db_sub.get_message(timeout=10)
            if db_notice is not None:
                rx_data = redis_util.get_rx(db_redis)
                attitude_data = redis_util.get_attitude(db_redis)
                rx_rc_data = rx_comm.read_data()[:6] # only take first 6 channels
                redis_util.set_cmd(db_redis, cmd_data)
    except KeyboardInterrupt:
        print("\nInterrupt received: stopping attitude control...")

if __name__ == "__main__":
    run_attitude_control()
