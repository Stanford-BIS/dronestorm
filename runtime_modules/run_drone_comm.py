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

def print_drone_comm_header():
    """Utility to print the header for drone comm"""
    print(
        "      Attitude       | " +
        "                  IMU                  | " +
        "                                  Command ")
    print(
        "  Roll  Pitch    Yaw | " +
        "   ax    ay    az  dRoll dPitch   dYaw |" +
        "     Throttle        dRoll       dPitch         dYaw    " +
        "     AUX1         AUX2")

def print_drone_comm(attitude, imu, cmd, cmd_rc):
    """Utility to print drone comm data"""
    sys.stdout.write("%+6.1f %+6.1f %+6.1f | "%tuple(attitude))
    sys.stdout.write("%+5.f %+5.f %+5.f %+6.f %+6.f %+6.f | "%tuple(imu))
    sys.stdout.write(
        "%+6.3f(%4d) "%(cmd[0], cmd_rc[0]) +
        "%+6.3f(%4d) "%(cmd[1], cmd_rc[1]) +
        "%+6.3f(%4d) "%(cmd[2], cmd_rc[2]) +
        "%+6.3f(%4d) "%(cmd[3], cmd_rc[3]) +
        "%+6.3f(%4d) "%(cmd[4], cmd_rc[4]) +
        "%+6.3f(%4d) "%(cmd[5], cmd_rc[5]) +
        "\r")
    sys.stdout.flush()

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
            redis_util.set_cmd_rc(db_redis, drone_comm.rc_data)
            print_drone_comm(attitude, imu, cmd, drone_comm.rc_data)

    except KeyboardInterrupt:
        print("Interrupt received: closing port and resetting settings...")
        drone_comm.exit()

if __name__ == "__main__":
    run_drone_comm()
