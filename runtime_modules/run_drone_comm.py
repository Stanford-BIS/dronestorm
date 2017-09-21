"""Communicate with flight control board and redis database during runtime

run from terminal with
`python run_drone_comm.py`
"""
from __future__ import print_function
import sys
from dronestorm.comm import DroneComm
import dronestorm.comm.drone as drone
from dronestorm.redis_util import DBRedis
import dronestorm.redis_util as redis_util
from dronestorm.log_util import print_drone_comm_header, print_drone_comm

def run_drone_comm():
    """Function to forward rc commands to the flight control board
    
    Reads IMU data from the flight control board
    Writes IMU data to the redis database
    
    Reads command data (REDIS_CMD_*) from redis database
    Writes command data to the flight control board
    """
    db_redis = DBRedis()
    db_redis.reset_db()
    drone_comm = DroneComm()
    print_drone_comm_header()
    try:
        while True:
            drone_comm.update_attitude()
            drone_comm.update_imu()
            attitude = drone.get_attitude(drone_comm)
            imu = drone.get_imu(drone_comm)
            redis_util.set_attitude(db_redis, attitude)
            redis_util.set_imu(db_redis, imu)

            cmd = redis_util.get_cmd(db_redis)
            drone.set_signals(drone_comm, cmd)
            drone_comm.send_rc(drone_comm.rc_data)
            print_drone_comm(attitude, imu, cmd, drone_comm.rc_data)

    except KeyboardInterrupt:
        print("Interrupt received: closing port and resetting settings...")
        drone_comm.exit()

if __name__ == "__main__":
    run_drone_comm()
