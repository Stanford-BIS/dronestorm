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
        drone.update_attitude()
        aux1 = float(r.get('aux1'))

        if(aux1 < MID_WIDTH):
            # Manual control

            yaw = float(r.get('m_yaw'))
            pitch = float(r.get('m_pitch'))
            roll = float(r.get('m_roll'))
            thr = float(r.get('m_thr'))

            drone.set_pitch_pwidth(pitch)
            drone.set_roll_pwidth(roll)
            drone.set_yaw_pwidth(yaw)
            drone.set_thr_pwidth(thr)

            curr_roll = drone.get_roll()
            curr_pitch = drone.get_pitch()
            curr_yaw = drone.get_yaw()

            sys.stdout.write(
                "roll:%6.5f pitch:%5.5f yaw:%5.5f\r" %
                (curr_roll, curr_pitch, curr_yaw))
            sys.stdout.flush()

        else:
            # Autonomy
            yaw = float(r.get('a_yaw'))
            pitch = float(r.get('a_pitch'))
            roll = float(r.get('a_roll'))
            thr = float(r.get('a_thr'))

            drone.set_pitch_pwidth(pitch)
            drone.set_roll_pwidth(roll)
            drone.set_yaw_pwidth(yaw)
            drone.set_thr_pwidth(thr)

            sys.stdout.write(
                "roll:%.5f pitch:%.5f yaw:%.5f\r"%
                (drone.get_roll(), drone.get_pitch(), drone.get_yaw()))
            sys.stdout.flush()

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
