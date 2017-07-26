from DroneControl import DroneComm, PID
import sys
import time
import redis
import numpy as np

# Flight stabilization app using the DroneControl Library
# Current roll and pitch values are acquired from the flight controller
# then roll and pitch rates are set to correct the error

drone = DroneComm()

# yaw parameters
Kp_yaw  = 0.075
Kd_yaw  = -0.0003
Ki_yaw  = 0.

out_yaw_limit = 1.0

error_yaw = 0
d_error_yaw = 0

# roll parameters
Kp_roll  = 0.02*0.8
Kd_roll  = 0.0001
Ki_roll  = 0.

out_roll_limit = 1.0

error_roll = 0
d_error_roll = 0

# pitch parameters
Kp_pitch  = 0.02*0.6
Kd_pitch  = 0.0001
Ki_pitch  = 0.

out_pitch_limit = 1.0

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
yaw0 = drone.get_yaw()
roll0 = drone.get_roll()
pitch0 = drone.get_pitch()

yaw_controller = PID(
    kp = Kp_yaw, kd = Kd_yaw, ki = Ki_yaw,
    get_state = drone.get_yaw, get_dstate = drone.get_dyaw,
    get_ref = lambda:yaw0,
    center_error = center_error,
    out_limit=out_yaw_limit)

roll_controller = PID(
    kp = Kp_roll, kd = Kd_roll, ki = Ki_roll,
    get_state = drone.get_roll, get_dstate = drone.get_droll,
    get_ref = lambda:roll0,
    center_error = center_error,
    out_limit=out_roll_limit)

pitch_controller = PID(
    kp = Kp_pitch, kd = Kd_pitch, ki = Ki_pitch,
    get_state = drone.get_pitch, get_dstate = drone.get_dpitch,
    get_ref = lambda:pitch0,
    center_error = center_error,
    out_limit=out_pitch_limit)

################ run the control

print(
    '   ry     y    dy     oy |' +
    '    rr     r    dr     or |' +
    '    rp     p    dp     op')
try:
    while (True):
        # update telemetry data
        drone.update_imu()
        drone.update_attitude()
        #step controllers
        output_yaw = yaw_controller.step()
        sys.stdout.write(
            "%5.1f %5.1f %5.0f %6.3f | "%
            (yaw_controller.ref, yaw_controller.state, yaw_controller.dstate,
             output_yaw))

        output_roll = roll_controller.step()
        sys.stdout.write(
            "%5.1f %5.1f %5.0f %6.3f | "%
            (roll_controller.ref, roll_controller.state,
             roll_controller.dstate, output_roll))

        output_pitch = pitch_controller.step()
        sys.stdout.write(
            "%5.1f %5.1f %5.0f %6.3f\r"%
            (pitch_controller.ref, pitch_controller.state,
             pitch_controller.dstate, output_pitch))

        sys.stdout.flush()

        # Set corrective rates
        drone.set_yaw_rate(output_yaw)
        drone.set_roll_rate(output_roll)
        drone.set_pitch_rate(output_pitch)

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
