from DroneControl import DroneComm, PID
import sys
import time
import redis
import numpy as np

# Flight stabilization app using the DroneControl Library
# Current roll and pitch values are acquired from the flight controller
# then pulse width signals are determined to fix the error

drone = DroneComm()

r = redis.StrictRedis()
r.set('yaw0', drone.get_yaw())
r.set('z0', 0.0)
r.set('y0', 0.0)

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

#roll parameters
Ku_roll = 0.2
Tu_roll = 0.4
# Proportion coefficients: how strongly the error should be corrected
Kp_roll  = Ku_roll * 0.45
Kd_roll  = 0.2*Kp_roll * Tu_roll * .125
Ki_roll  = 0.

out_limit = 1.0

error_roll = 0
int_error_roll = 0
int_error_limit = out_limit
d_error_roll = 0

#pitch parameters
Ku_pitch = 0.2
Tu_pitch = 0.4
# Proportion coefficients: how strongly the error should be corrected
Kp_pitch  = Ku_pitch * 0.45
Kd_pitch  = 0.2*Kp_pitch * Tu_pitch * .125
Ki_pitch  = 0.

out_limit = 1.0

error_pitch = 0
int_error_pitch = 0
int_error_limit = out_limit
d_error_pitch = 0

############### Start program by placing drone on a flat surface to ensure accurate

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

roll_controller = PID(
    Kp_roll, Kd_roll, Ki_roll,
    lambda : z0, r.get_z, drone.get_roll,    #state is z coordinate from camera, derivative is approx the roll value
    lambda x: x, out_limit)                     #####CHECK used lambda x: x for the center_error function (and for pitch)

pitch_controller = PID(
    Kp_pitch, Kd_pitch, Ki_pitch,
    lambda : y0, r.get_y, drone.get_pitch,   #state is y coordinate from camera, derivative is approx the pitch value
    lambda x: x, out_limit)

###############3 run the control

while r.get('angle') is None:
    time.sleep(1.0)
    print('waiting for dvs yaw data...')

try:
    while (True):
        # calibration Values
        yaw0 = drone.get_yaw()
        z0=r.get('z0')
        y0=r.get('y0')
        
        #step controllers
        output_yaw = yaw_controller.step()
        sys.stdout.write(
            "yaw0:%5.1f yaw:%5.1f dyaw:%5.0f output_yaw:%6.3f\r"%
            (yaw_controller.ref, yaw_controller.state, yaw_controller.dstate,
             output_yaw))

        output_roll = roll_controller.step()
        sys.stdout.write(
            "roll0:%5.1f roll:%5.1f droll:%5.0f output_roll:%6.3f\r"%
            (roll_controller.ref, roll_controller.state, roll_controller.dstate,
             output_roll))

        output_pitch = pitch_controller.step()
        sys.stdout.write(
            "pitch0:%5.1f pitch:%5.1f dpitch:%5.0f output_pitch:%6.3f\r"%
            (pitch_controller.ref, pitch_controller.state, pitch_controller.dstate,
             output_pitch))

        sys.stdout.flush()

        # Set corrective rates
        drone.set_yaw_rate(output_yaw)
        drone.set_roll_rate(output_roll)        ########CHECK correct way of setting roll and pitch??
        drone.set_pitch_rate(output_pitch)

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
