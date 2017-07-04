"""Monitor drone sensor values
"""
from DroneControl import DroneControl
import sys

drone = DroneControl()

try:
    while (True):
        # Fetching current pitch/roll values
        roll = drone.getRoll()
        pitch = drone.getPitch()
        yaw = drone.getYaw()
        sys.stdout.write(
            "roll:%6.1f pitch:%5.1f yaw:%5.1f\r"%(roll, pitch, yaw))
        sys.stdout.flush()
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    print("")
    drone.exit()
