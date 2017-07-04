from DroneControl import DroneControl

# Flight stabilization app using the DroneControl Library
# Current roll and pitch values are acquired from the flight controller
# then pulse width signals are determined to fix the error

drone = DroneControl()

# Proportion coefficients: how strongly the error should be corrected
K_pitch = .01
K_roll = .01

# Start program by placing drone on a flat surface to ensure accurate
# calibration Values
desired_Pitch = drone.getData("angy")
desired_Roll = drone.getData("angx")

try:
    while (True):
        # Fetching current pitch/roll values
        pitch = drone.getData("angy")
        roll = drone.getData("angx")

        # Determining error between desired pitch/roll and actual pitch/roll
        error_pitch = desired_Pitch - pitch
        error_roll =  desired_Roll - roll

        # Output roll/pitch is calculated in terms of pulse width
        # Valid ranges: 1.1ms -> 1.9ms; 1.5ms is MED_WIDTH
        output_pitch = K_pitch * error_pitch + drone.MED_WIDTH
        output_roll = K_roll * error_roll + drone.MED_WIDTH

        # Setting corrected PWM pulse widths
        drone.setPitch(output_pitch)
        drone.setRoll(output_roll)

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
