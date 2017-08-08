"""Monitor drone sensor values
"""
from DroneControl import DroneComm
import sys

drone = DroneComm(pwm_ctrl=False)

print('  roll  droll | pitch dpitch |   yaw   dyaw |    ax     ay     az')
try:
    while (True):
        # Fetching current pitch/roll values
        drone.update_attitude()
        drone.update_imu()
        roll = drone.get_roll()
        pitch = drone.get_pitch()
        yaw = drone.get_yaw()
        droll = drone.get_droll()
        dpitch = drone.get_dpitch()
        dyaw = drone.get_dyaw()

        ax = drone.get_ax()
        ay = drone.get_ay()
        az = drone.get_az()

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
