from DroneControl import DroneComm
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

# run the control
try:
    while (True):
        yaw = drone.get_yaw()
        # Error between desired and actual roll/pitch
        error_yaw = yaw0 - yaw
        if error_yaw > 180:
            error_yaw -= 360
        elif error_yaw < -180:
            error_yaw += 360

        dyaw = drone.get_dyaw()
        d_error_yaw = dyaw

        int_error_yaw += error_yaw
        int_error_yaw = np.clip(int_error_yaw,
            -int_error_limit, int_error_limit)

        output_yaw = np.clip(
            Kp_yaw * error_yaw + Ki_yaw * int_error_yaw + Kd_yaw * d_error_yaw,
            -out_limit, out_limit)

        sys.stdout.write(
            "yaw0:%5.1f yaw:%5.1f dyaw:%5.0f output_yaw:%6.3f\r"%
            (yaw0, yaw, dyaw, output_yaw))
        sys.stdout.flush()

        # Set corrective rates
        drone.set_yaw_rate(output_yaw)
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
