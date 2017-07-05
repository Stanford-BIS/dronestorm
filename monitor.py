"""Monitor drone sensor values
"""
from DroneControl import DroneComm
import sys

drone = DroneComm()

try:
    while (True):
        # Fetching current pitch/roll values
        roll = drone.get_roll()
        pitch = drone.get_pitch()
        yaw = drone.get_yaw()
        sys.stdout.write(
            "roll:%6.1f pitch:%5.1f yaw:%5.1f\r"%(roll, pitch, yaw))
        sys.stdout.flush()
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    print("")
    drone.exit()
