from droneControl import DroneControl

# Flight stabilization app using the DroneControl Library
drone = DroneControl()

drone.reset()
time.sleep(1)

K_pitch = .01
K_roll = .01

desired_Pitch = des_p
desired_Roll = des_r

try:
    while (True):
        pitch = drone.getData(board, "angy")
        roll = drone.getData(board, "angx")

        error_pitch = desired_Pitch - pitch
        error_roll =  desired_Roll - roll

        output_pitch = K_pitch * error_pitch + pitch
        output_roll = K_roll * error_roll + roll

        drone.setPitch(output_pitch)
        drone.setRoll(output_roll)
        drone.setYaw(1.5)

except (KeyboardInterrupt, SystemExit):
    board.closeSerial()
