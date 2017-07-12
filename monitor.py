"""Monitor drone sensor values
"""
from DroneControl import DroneComm
import sys

drone = DroneComm(pwm_ctrl=False)

try:
    while (True):
        # Fetching current pitch/roll values
        roll = drone.get_roll()
        pitch = drone.get_pitch()
        yaw = drone.get_yaw()

        ax = drone.get_ax()
        ay = drone.get_ay()
        az = drone.get_az()
        gx = drone.get_gx()
        gy = drone.get_gy()
        gz = drone.get_gz()

        sys.stdout.write(
            "roll:%6.1f pitch:%5.1f yaw:%5.1f "%(roll, pitch, yaw) +
            "ax:%f ay:%f az:%f "%(ax, ay, az) +
            "gx:%f gy:%f gz:%f\r"%(gx, gy, gz))
        sys.stdout.flush()
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    print("")
    drone.exit()
