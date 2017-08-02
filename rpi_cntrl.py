import RPi.GPIO as GPIO
import time, sys, redis
import numpy as np
from DroneControl import DroneComm

host = '127.0.0.1'
r = redis.StrictRedis(host=host)
drone = DroneComm()

MID_WIDTH = 0.0015



try:
    while(True):

        aux1 = float(r.get('aux1'))

        if(aux1 < MID_WIDTH):
            # Manual control

            yaw = float(r.get('yaw'))
            pitch = float(r.get('pitch'))
            roll = float(r.get('roll'))

            drone.set_pitch_pwidth(pitch)
            drone.set_roll_pwidth(roll)
            drone.set_yaw_pwidth(yaw)

            curr_roll = drone.get_roll()
            curr_pitch = drone.get_pitch()
            curr_yaw = drone.get_yaw()

            sys.stdout.write(
                "roll:%6.5f pitch:%5.5f yaw:%5.5f\r" %
                (roll, pitch, yaw))
            sys.stdout.flush()

        # else:
        #     # Autonomy
        #     yaw = float(r.get('yaw'))
        #     pitch = float(r.get('pitch'))
        #     roll = float(r.get('roll'))
        #
        #     drone.set_pitch_pwidth(pitch)
        #     drone.set_roll_pwidth(roll)
        #     drone.set_yaw_pwidth(yaw)
        #
        #     sys.stdout.write(
        #         "roll:%.5f pitch:%.5f yaw:%.5f\r"%
        #         (drone.get_roll(), drone.get_pitch(), drone.get_yaw()))
        #     sys.stdout.flush()

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
