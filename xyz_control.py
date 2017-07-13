from DroneControl import DroneComm, PID, calibrate_april_imu_yaw
import sys
import time
import redis
import numpy as np

# Flight stabilization app using the DroneControl Library
# Current roll and pitch values are acquired from the flight controller
# then pulse width signals are determined to fix the error

drone = DroneComm()
r = redis.StrictRedis()

try:
    yaw_cal = calibrate_april_imu_yaw(r, drone)
except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
    exit()

# yaw_cal = 0
x0 = float(r.get('y'))
y0 = float(r.get('z'))
r.set('rx', 0.0)
r.set('ry', 0.0)

# x parameters
# Proportion coefficients: how strongly the error should be corrected
Kp_x  = 10.
Kd_x  = 0.
Ki_x  = 0.

out_x_limit = 10.0

error_x= 0
d_error_x= 0

# y parameters
# Proportion coefficients: how strongly the error should be corrected
Kp_y  = 10.
Kd_y  = 0.
Ki_y  = 0.

out_y_limit = 10.0

error_y= 0
d_error_y= 0
 
# yaw parameters
# Zigler Nichols estimated tuning parameters
Ku_yaw = 0.2
Tu_yaw = 0.4
# Proportion coefficients: how strongly the error should be corrected
Kp_yaw  = Ku_yaw * 0.45
Kd_yaw  = 0.2*Kp_yaw * Tu_yaw * .125
Ki_yaw  = 0.

out_yaw_limit = 0.

error_yaw = 0
d_error_yaw = 0

# roll parameters
# Proportion coefficients: how strongly the error should be corrected
Kp_roll  = 0.02*0.8
Kd_roll  = -0.0001
Ki_roll  = 0.

out_roll_limit = 1.0

error_roll = 0
int_error_roll = 0
d_error_roll = 0

# pitch parameters
# Proportion coefficients: how strongly the error should be corrected
Kp_pitch  = 0.02*.6
Kd_pitch  = 0.0001
Ki_pitch  = 0.

out_pitch_limit = 0.0

error_pitch = 0
d_error_pitch = 0

######### Start program by placing drone on a flat surface to calibrate origin

def center_error(error):
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

drone.update_imu()
drone.update_attitude()
roll0 = drone.get_roll()
pitch0 = drone.get_pitch()
yaw0 = drone.get_yaw()

x_controller = PID(
    Kp_x, Kd_x, Ki_x,
    lambda : x0 + float(r.get('rx')), lambda : float(r.get('y')), lambda : 0,
    lambda e : e, out_x_limit)

y_controller = PID(
    Kp_y, Kd_y, Ki_y,
    lambda : y0 + float(r.get('ry')), lambda : float(r.get('z')), lambda : 0,
    lambda e : e, out_y_limit)

def compute_roll0():
    r0 = -y_controller.step()
    return r0

def compute_pitch0():
    p0 = x_controller.step()
    return p0

yaw_controller = PID(
    kp = Kp_yaw, kd = Kd_yaw, ki = Ki_yaw,
    get_state = drone.get_yaw, get_dstate = lambda: -drone.get_dyaw(),
    get_ref = lambda:yaw0,
    center_error = center_error,
    out_limit=out_yaw_limit)

roll_controller = PID(
    kp = Kp_roll, kd = Kd_roll, ki = Ki_roll,
    get_state = drone.get_roll, get_dstate = drone.get_droll,
    get_ref = compute_roll0,
    center_error = center_error,
    out_limit=out_roll_limit)

pitch_controller = PID(
    kp = Kp_pitch, kd = Kd_pitch, ki = Ki_pitch,
    get_state = drone.get_pitch, get_dstate = drone.get_dpitch,
    get_ref = compute_pitch0,
    out_limit=out_pitch_limit)

################ run the control ##############################

print(
    '   ry     y    dy     oy |' +
    '    rr     r    dr     or |' +
    '    rp     p    dp     op |'+
    '    xr     x    yr     y ')
try:
    while (True):
        # update telemetry data
        drone.update_imu()
        drone.update_attitude()
        # step controllers
        output_yaw = yaw_controller.step()
        sys.stdout.write(
            "%5.1f %5.1f %5.0f %6.3f | "%
            (yaw_controller.ref, yaw_controller.state, yaw_controller.dstate,
             output_yaw))

        output_roll = roll_controller.step()
        sys.stdout.write(
            "%5.1f %5.1f %5.0f %6.3f | "%
            (roll_controller.ref, roll_controller.state,
             roll_controller.dstate,output_roll))

        output_pitch = pitch_controller.step()
        sys.stdout.write(
            "%5.1f %5.1f %5.0f %6.3f | "%
            (pitch_controller.ref, pitch_controller.state,
             pitch_controller.dstate,
             output_pitch))
        sys.stdout.flush()

        sys.stdout.write(
            "%5.2f %5.2f %5.2f %5.2f\r"%
            (x_controller.ref, x_controller.state,
             y_controller.ref, y_controller.state))
        sys.stdout.flush()

        # Set corrective rates
        drone.set_yaw_rate(output_yaw)
        drone.set_roll_rate(output_roll)
        drone.set_pitch_rate(output_pitch)

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
