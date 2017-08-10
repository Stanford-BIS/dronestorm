from dronestorm import DroneComm
import sys
import time

# Flight stabilization app using the DroneControl Library
# Current roll and pitch values are acquired from the flight controller
# then pulse width signals are determined to fix the error

drone = DroneComm(roll_pwm_trim=-5, pitch_pwm_trim=4)
# drone = DroneComm()

# Proportion coefficients: how strongly the error should be corrected
K_roll  = 4 * 1./180.
K_pitch = 4 * 1./90.

# Start program by placing drone on a flat surface to ensure accurate
# calibration Values
drone.update_attitude()
desired_roll = drone.get_roll()
desired_pitch = drone.get_pitch()

print("desired_roll:%f desired_pitch:%f"%(desired_roll, desired_pitch))

try:
    while (True):
        # Fetching current roll/pitch values
        drone.update_attitude()
        roll = drone.get_roll()
        pitch = drone.get_pitch()

        # Error between desired and actual roll/pitch
        error_roll =  desired_roll - roll
        error_pitch = desired_pitch - pitch

        output_roll = K_roll * error_roll
        output_pitch = K_pitch * error_pitch

        sys.stdout.write(
            "roll:%6.1f pitch:%5.1f output_roll:%6.3f output_pitch:%6.3f\r"%
            (roll, pitch, output_roll, output_pitch))
        sys.stdout.flush()

        # Set corrective rates
        drone.set_pitch_rate(output_pitch)
        drone.set_roll_rate(output_roll)

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
