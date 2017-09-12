"""Monitor drone IMU sensor values"""
from __future__ import absolute_import
import sys
from dronestorm import DroneComm

drone = DroneComm(pwm_ctrl=False, port='/dev/ttyACM0')

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
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    print("")
    drone.exit()
