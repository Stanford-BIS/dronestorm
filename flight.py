from DroneControl import DroneComm

# Flight stabilization app using the DroneControl Library
# Current roll and pitch values are acquired from the flight controller
# then pulse width signals are determined to fix the error

drone = DroneComm()

# Proportion coefficients: how strongly the error should be corrected
K_pitch = .01
K_roll = .01

# Start program by placing drone on a flat surface to ensure accurate
# calibration Values
desired_Pitch = drone.get_data("angy")
desired_Roll = drone.get_data("angx")

try:
    while (True):
        # Fetching current pitch/roll values
        pitch = drone.get_data("angy")
        roll = drone.get_data("angx")

        # Determining error between desired pitch/roll and actual pitch/roll
        error_pitch = desired_Pitch - pitch
        error_roll =  desired_Roll - roll

        # Output roll/pitch is calculated in terms of pulse width
        # Valid ranges: 1.1ms -> 1.9ms; 1.5ms is MED_WIDTH
        output_pitch = K_pitch * error_pitch + drone.MED_WIDTH
        output_roll = K_roll * error_roll + drone.MED_WIDTH

        # Setting corrected PWM pulse widths
        drone.set_pitch(output_pitch)
        drone.set_roll(output_roll)

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
