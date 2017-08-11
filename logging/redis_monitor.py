"""Monitors drone attitude values writes them to a redis database

Run a redis server on a computer attached to the network
    The redis server needs to be configured to accept connections from other
    computers
Note the ip address or hostname of the computer running the redis server
"""
from dronestorm import DroneComm
import redis
import sys

host = 'localhost' # ip or hostname of computer running the redis server
r = redis.StrictRedis(host=host)
drone = DroneComm(pwm_ctrl=False)

print('  roll  droll | pitch dpitch |   yaw   dyaw |    ax     ay     az')
try:
    while (True):
        # Fetching current pitch/roll values
        drone.update_attitude()
        drone.update_imu()
        roll = drone.attitude['roll']
        pitch = drone.attitude['pitch']
        yaw = drone.attitude['yaw']
        droll = drone.imu['droll']
        dpitch = drone.imu['dpitch']
        dyaw = drone.imu['dyaw']
        ax = drone.imu['ax']
        ay = drone.imu['ay']
        az = drone.imu['az']
        sys.stdout.write(
            "%6.1f  %5.0f | "%(roll, droll) +
            "%5.1f  %5.0f | "%(pitch, dpitch) +
            "%5.1f  %5.0f | "%(yaw, dyaw) +
            "%5.0f  %5.0f  %5.0f\r"%(ax, ay, az))
        sys.stdout.flush()

        r.set('roll', roll)
        r.set('pitch', pitch)
        r.set('yaw', yaw)
        r.set('droll', droll)
        r.set('dpitch', dpitch)
        r.set('dyaw', dyaw)
        r.set('ax', ax)
        r.set('ay', ay)
        r.set('az', az)

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    print("")
    drone.exit()
