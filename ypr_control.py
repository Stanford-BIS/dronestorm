from DroneControl import DroneComm
import sys
import time
import redis
import numpy as np
import math

# Flight stabilization app using the DroneControl Library
# Current roll and pitch values are acquired from the flight controller
# then pulse width signals are determined to fix the error

drone = DroneComm(port=None)
dt = 0.01

# Proportion coefficients: how strongly the error should be corrected

Kp_yaw  = 0.35
Kd_yaw  = 0.2
Ki_yaw  = 0.

out_limit_yaw = 1.0

error_yaw = 0
int_error_yaw = 0
int_error_limit_yaw = out_limit_yaw
d_error_yaw = 0
d_error_prev_yaw = np.zeros(3)
d_error_idx = 0
d_error_yaw_filt = 0.9
error_yaw_prev = 0


#roll variables
Kp_roll  = 0.35
Kd_roll  = 0.2
Ki_roll  = 0.

out_limit_roll = 1.0

error_roll = 0
int_error_roll = 0
int_error_limit_roll = out_limit_roll
d_error_roll = 0
d_error_prev_roll = np.zeros(3)
d_error_roll_filt = 0.9
error_roll_prev = 0

#Pitch variables
Kp_pitch  = 0.35
Kd_pitch = 0.2
Ki_pitch = 0.

out_limit_pitch = 1.0

error_pitch= 0
int_error_pitch= 0
int_error_limit_pitch = out_limit_pitch
d_error_pitch= 0
d_error_prev_pitch = np.zeros(3)
d_error_pitch_filt = 0.9
error_pitch_prev = 0

# Start program by placing drone on a flat surface to ensure accurate
# calibration Values

r = redis.StrictRedis()
r.set('yaw0', 0.0)
r.set('y0', 0.0)
r.set('x0', 0.0)

while r.get('angle') is None:
    time.sleep(1.0)
    print('waiting for dvs yaw data...')

try:
    while (True):
        yaw0 = float(r.get('yaw0'))
        yaw = -float(r.get('yaw'))
    # Error between desired and actual roll/pitch
#       yaw = yaw + dyaw * dt  #add the imu derivative data to interpolate between frames
        error_yaw_prev = error_yaw
        error_yaw = yaw0 - yaw

        d_error_prev_yaw[d_error_idx] = (error_yaw - error_yaw_prev) / dt
        d_error_idx += 1
        d_error_idx %= 3
        d_error_yaw = np.mean(d_error_prev_yaw)

        int_error_yaw += error_yaw
        int_error_yaw = np.clip(int_error_yaw,
            -int_error_limit_yaw, int_error_limit_yaw)

        output_yaw = np.clip(
            Kp_yaw * error_yaw + Ki_yaw * int_error_yaw + Kd_yaw * d_error_yaw,
            -out_limit_yaw, out_limit_yaw)

        sys.stdout.write(
            "yaw0:%7.2f yaw:%5.1f output_yaw:%6.3f\r"%
            (yaw0, yaw, output_yaw))
        sys.stdout.flush()

        # Set corrective rates
        drone.set_yaw_rate(-output_yaw)#use - when apriltag is on the bottom

        #############roll
        roll0 = float(r.get('y0'))
        y = -float(r.get('y'))
        x = -float(r.get('x'))
        #change axes to drone axis
        roll = -x*math.sin(yaw)+y*math.cos(yaw)

        # Error between desired and actual roll/pitch
#       roll = roll + droll * dt  #add the imu derivative data to interpolate between frames
        error_roll_prev = error_roll
        error_roll = roll0 - roll

        d_error_prev_roll[d_error_idx] = (error_roll - error_roll_prev) / dt
        d_error_roll = np.mean(d_error_prev_roll)

        int_error_roll += error_roll
        int_error_roll = np.clip(int_error_roll,
            -int_error_limit_roll, int_error_limit_roll)

        output_roll = np.clip(
            Kp_roll * error_roll + Ki_roll * int_error_roll + Kd_roll * d_error_roll,
            -out_limit_roll, out_limit_roll)

        sys.stdout.write(
            "roll0:%7.2f roll:%5.1f output_roll:%6.3f\r"%
            (roll0, roll, output_roll))
        sys.stdout.flush()

        # Set corrective rates
        drone.set_roll_rate(output_roll)

        #############pitch
        pitch0 = float(r.get('x0'))
 
        #change axes to drone axis
        pitch = x*math.cos(yaw)+y*math.sin(yaw)

        # Error between desired and actual pitch/pitch
#       pitch = pitch + dpitch * dt  #add the imu derivative data to interpolate between frames
        error_pitch_prev = error_pitch
        error_pitch = pitch0 - pitch

        d_error_prev_pitch[d_error_idx] = (error_pitch - error_pitch_prev) / dt
        d_error_pitch = np.mean(d_error_prev_pitch)

        int_error_pitch += error_pitch
        int_error_pitch = np.clip(int_error_pitch,
            -int_error_limit_pitch, int_error_limit_pitch)

   output_pitch = np.clip(
            Kp_pitch * error_pitch + Ki_pitch * int_error_pitch + Kd_pitch * d_error_pitch,
            -out_limit_pitch, out_limit_pitch)

        sys.stdout.write(
            "pitch0:%7.2f pitch:%5.1f output_pitch:%6.3f\r"%
            (pitch0, pitch, output_pitch))
        sys.stdout.flush()

        # Set corrective rates
        drone.set_pitch_rate(output_pitch)

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()



