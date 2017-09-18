"""Communicate with flight control board and redis database during runtime

Reads IMU data from the flight control board
Writes IMU data to the redis database

Reads command data (REDIS_CMD_*) from redis database
Writes command data to the flight control board

run from terminal with
`python run_drone_comm.py`
"""
from __future__ import print_function
import sys
from dronestorm.comm import DroneComm
import dronestorm.comm.drone as drone
from dronestorm.redis_util import DBRedis
import dronestorm.redis_util as redis_util

def run_drone_comm():
    """Function to forward rc commands to the flight control board"""
    db_redis = DBRedis()
    db_redis.reset_db()
    drone_comm = DroneComm()
    print("Throttle     Roll    Pitch      Yaw     AUX1     AUX2")
    try:
        while True:
            # drone_comm.update_attitude()
            drone_comm.update_imu()
            # attitude_data = drone.get_attitude(drone_comm)
            imu_data = drone.get_imu(drone_comm)
            # redis_util.set_attitude()
            redis_util.set_imu()

            cmd_data = redis_util.get_cmd(db_redis)
            drone.set_signal(drone_comm, cmd_data)
            drone_comm.send_rc(drone_comm.rc_data)

            sys.stdout.write(
                "    %4d     %4d     %4d     %4d     %4d     %4d\r"%
                tuple(rc_data[:6]))
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("Interrupt received: closing port and resetting settings...")
        drone_comm.exit()

if __name__ == "__main__":
    run_drone_comm()
