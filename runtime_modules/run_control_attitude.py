"""Compute attitude controlling outputs during runtime

run from terminal with
`python run_control_attitude.py`
"""
from __future__ import print_function
from dronestorm.comm.redis_util import DBRedis
import dronestorm.comm.redis_util as redis_util
from  dronestorm.comm.redis_util import REDIS_RX_CHANNEL, REDIS_ATTITUDE_CHANNEL, REDIS_IMU_CHANNEL
from dronestorm.control.attitude import AttitudePD

def run_attitude_control():
    """Function to compute the control signals for attitude control

    Reads IMU data from the redis database
    Reads receiver data from the redis database
    Writes command data to the redis database
    """
    attitude_controller = AttitudePD(
        roll_kp=0.04, roll_kd=0.000001, pitch_kp=0.04, pitch_kd=0.000001)

    db_redis = DBRedis()
    db_sub = db_redis.subscribe([REDIS_RX_CHANNEL, REDIS_ATTITUDE_CHANNEL, REDIS_IMU_CHANNEL])

    try:
        print("Running attitude control...Ctrl-c to stop")
        while True:
            # check for new receiver or attitude data
            db_notice = db_sub.get_message(timeout=10)
            if db_notice is not None:
                rx_data = redis_util.get_rx(db_redis)
                attitude = redis_util.get_attitude(db_redis)
                imu = redis_util.get_imu(db_redis)

                droll_cmd, dpitch_cmd = attitude_controller.step(
                    attitude[:2], imu[:2], rx_data[1:3], [0., 0.])

                throttle = rx_data[0]
                dyaw_cmd = rx_data[3]
                aux1, aux2 = rx_data[4:]
                cmd = [throttle, droll_cmd, dpitch_cmd, dyaw_cmd, aux1, aux2]
                redis_util.set_cmd(db_redis, cmd)
    except KeyboardInterrupt:
        print("\nInterrupt received: stopping attitude control...")

if __name__ == "__main__":
    run_attitude_control()
