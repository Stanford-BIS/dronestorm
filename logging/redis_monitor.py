"""Monitors drone attitude values writes them to a redis database

Run a redis server on a computer attached to the network
    The redis server needs to be configured to accept connections from other
    computers
Note the ip address or hostname of the computer running the redis server
"""
from dronestorm import DroneComm
import redis
import sys

host = '192.168.0.165' # ip or hostname of computer running the redis server
r = redis.StrictRedis(host=host)
drone = DroneComm()

try:
    while (True):
        # Fetching current pitch/roll values
        drone.update_attitude()
        drone.update_imu()
        roll = drone.get_roll()
        pitch = drone.get_pitch()
        yaw = drone.get_yaw()
        sys.stdout.write(
            "roll:%6.1f pitch:%5.1f yaw:%5.1f\r"%(roll, pitch, yaw))
        sys.stdout.flush()
        r.set('roll', roll)
        r.set('pitch', pitch)
        r.set('yaw', yaw)

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    print("")
    drone.exit()
