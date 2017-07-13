from DroneControl import DroneComm, PID
import sys
import time
import redis
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

out_yaw_limit = 1.0

error_yaw = 0
int_error_yaw = 0
int_error_yaw_limit = out_yaw_limit
d_error_yaw = 0

#roll parameters
# Proportion coefficients: how strongly the error should be corrected
Kp_roll  = 0.02*0.8
Kd_roll  = -0.0001
Ki_roll  = 0.

out_roll_limit = 1.0

error_roll = 0
int_error_roll = 0
int_error_roll_limit = out_roll_limit
d_error_roll = 0

#pitch parameters
# Proportion coefficients: how strongly the error should be corrected
Kp_pitch  = 0.02*0.6
Kd_pitch  = 0.0001
Ki_pitch  = 0.
# Kp_pitch  = Ku_pitch * 0.45
# Kd_pitch  = 0.2*Kp_pitch * Tu_pitch * .125
# Ki_pitch  = 0.

out_pitch_limit = 1.0

error_pitch = 0
int_error_pitch = 0
int_error_pitch_limit = out_pitch_limit
d_error_pitch = 0

############### Start program by placing drone on a flat surface to ensure accurate

def center_error(error):
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

yaw0 = drone.get_yaw()
roll0 = drone.get_roll()
pitch0 = drone.get_pitch()

yaw_controller = PID(
    Kp_yaw, Kd_yaw, Ki_yaw,
    lambda:yaw0, drone.get_yaw, drone.get_dyaw,
    center_error, out_yaw_limit)

roll_controller = PID(
    Kp_roll, Kd_roll, Ki_roll,
    lambda:roll0, drone.get_roll, drone.get_droll,
    center_error, out_roll_limit)

pitch_controller = PID(
    Kp_pitch, Kd_pitch, Ki_pitch,
    lambda:pitch0, drone.get_pitch, drone.get_dpitch,
    lambda x: x, out_pitch_limit)

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
