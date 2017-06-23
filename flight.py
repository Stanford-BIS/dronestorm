from droneControl import DroneControl

# Flight stabilization app using the DroneControl Library

drone = DroneControl()

K_pitch = .01
K_roll = .01
desired_Pitch = 0
desired_Roll = 0

try:
    while (True):
        pitch = drone.getData("angy")
        roll = drone.getData("angx")

        error_pitch = desired_Pitch - pitch
        error_roll =  desired_Roll - roll

        output_pitch = K_pitch * error_pitch + pitch
        output_roll = K_roll * error_roll + roll

        drone.setPitch(output_pitch)
        drone.setRoll(output_roll)

except (KeyboardInterrupt, SystemExit):
    drone.exit()
