from DroneControl import DroneComm
import sys
import time
import redis
import numpy as np

# Flight stabilization app using the DroneControl Library
# Current roll and pitch values are acquired from the flight controller
# then pulse width signals are determined to fix the error

drone = DroneComm(port=None)
dt = 0.01

# Proportion coefficients: how strongly the error should be corrected
Kp_yaw  = 0.35
Kd_yaw  = 0.2
Ki_yaw  = 0.

out_limit = 1.0

error_yaw = 0
int_error_yaw = 0
int_error_limit = out_limit
d_error_yaw = 0
d_error_prev = np.zeros(3)
d_error_idx = 0
d_error_yaw_filt = 0.9
error_yaw_prev = 0

# Start program by placing drone on a flat surface to ensure accurate
# calibration Values

r = redis.StrictRedis()
r.set('yaw0', 0.0)

while r.get('angle') is None:
    time.sleep(1.0)
    print('waiting for dvs yaw data...')

try:
    while (True):
        yaw0 = float(r.get('yaw0'))
        yaw = -float(r.get('yaw'))
        # Error between desired and actual roll/pitch
        error_yaw_prev = error_yaw
        error_yaw = yaw0 - yaw

        d_error_prev[d_error_idx] = (error_yaw - error_yaw_prev) / dt
        d_error_idx += 1
        d_error_idx %= 3
        d_error_yaw = np.mean(d_error_prev)

        int_error_yaw += error_yaw
        int_error_yaw = np.clip(int_error_yaw,
            -int_error_limit, int_error_limit)

        output_yaw = np.clip(
            Kp_yaw * error_yaw + Ki_yaw * int_error_yaw + Kd_yaw * d_error_yaw,
            -out_limit, out_limit)

        sys.stdout.write(
            "yaw0:%7.2f yaw:%5.1f output_yaw:%6.3f\r"%
            (yaw0, yaw, output_yaw))
        sys.stdout.flush()

        # Set corrective rates
        drone.set_yaw_rate(output_yaw)
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
