from droneControl import DroneControl
from time import sleep

# Flight stabilization app using the DroneControl Library

drone = DroneControl()

K_pitch = .01
K_roll = .01
desired_Pitch = 4.8
desired_Roll = -1.7

try:
    while (True):
        pitch = drone.getData("angy")
        roll = drone.getData("angx")

        error_pitch = desired_Pitch - pitch
        error_roll =  desired_Roll - roll

        output_pitch = K_pitch * error_pitch + drone.MED_WIDTH
        output_roll = K_roll * error_roll + drone.MED_WIDTH

        print("Current Pitch: " + str(pitch))
        print("Output Pitch: " + str(output_pitch) + "\n")

        print("Current Roll: " + str(roll))
        print("Output Roll: " + str(output_roll) + "\n")

        drone.setPitch(output_pitch)
        drone.setRoll(output_roll)

except (KeyboardInterrupt, SystemExit):
    drone.exit()
