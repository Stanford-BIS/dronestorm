"""Estimate state from sensor measurements during runtime

run from terminal with
`python run_estimate_kalman.py`
"""
from __future__ import print_function
from dronestorm.comm.redis_util import DBRedis
import dronestorm.comm.redis_util as redis_util
from  dronestorm.comm.redis_util import (
    REDIS_IMU_CHANNEL, REDIS_SONAR_CHANNEL, REDIS_GPS)
from dronestorm.estimate.kalman import EKF

def run_estimate_kalman():
    """Function to compute the control signals for attitude control

    Reads IMU data from the redis database
    Reads sonar data from the redis database
    Reads gps data from the redis database
    Writes state estimate data to the redis database
    """
    R = np.eye(12)
    estimator = EKF(R)
    estimator.initialize(Zgps=0, Zimu=0, Zagl=0)

    db_redis = DBRedis()
    db_sub = db_redis.subscribe(
        [REDIS_IMU_CHANNEL, REDIS_SONAR_CHANNEL, REDIS_GPS_CHANNEL])

    try:
        print("Running state estimate...Ctrl-c to stop")
        while True:
            # check for new receiver or attitude data
            db_notice = db_sub.get_message(timeout=10)
            if db_notice is not None:
                rx_data = redis_util.get_rx(db_redis)
                imu = redis_util.get_imu(db_redis)
                agl = redis_util.get_sonar(db_redis)
                gps = redis_util.get_gps(db_redis)

                estimator.step()

                state_est = [x, y, z, roll, pitch, yaw, dx, dy, dz, omega_x, omega_y, omega_z]
                redis_util.set_cmd(db_redis, state_est)
    except KeyboardInterrupt:
        print("\nInterrupt received: stopping state estimation...")

if __name__ == "__main__":
    run_estimate_kalman()
