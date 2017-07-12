from DroneControl import DroneComm, PID
import sys
import time
import numpy as np

# Flight stabilization app using the DroneControl Library
# Current roll and pitch values are acquired from the flight controller
# then pulse width signals are determined to fix the error

drone = DroneComm()

# Zigler Nichols estimated tuning parameters
Ku_yaw = 0.2
Tu_yaw = 0.4
# Proportion coefficients: how strongly the error should be corrected
Kp_yaw  = Ku_yaw * 0.45
Kd_yaw  = 0.2*Kp_yaw * Tu_yaw * .125
Ki_yaw  = 0.

out_limit = 1.0

error_yaw = 0
int_error_yaw = 0
int_error_limit = out_limit
d_error_yaw = 0

# Start program by placing drone on a flat surface to ensure accurate
# calibration Values
yaw0 = drone.get_yaw()

def center_yaw_error(error_yaw):
    if error_yaw > 180:
        error_yaw -= 360
    elif error_yaw < -180:
        error_yaw += 360
    return error_yaw

yaw_controller = PID(
    Kp_yaw, Kd_yaw, Ki_yaw,
    lambda : yaw0, drone.get_yaw, drone.get_dyaw,
    center_yaw_error, out_limit)

# run the control
try:
    while (True):
        output_yaw = yaw_controller.step()
        sys.stdout.write(
            "yaw0:%5.1f yaw:%5.1f dyaw:%5.0f output_yaw:%6.3f\r"%
            (yaw_controller.ref, yaw_controller.state, yaw_controller.dstate,
             output_yaw))
        sys.stdout.flush()

        # Set corrective rates
        drone.set_yaw_rate(output_yaw)
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
