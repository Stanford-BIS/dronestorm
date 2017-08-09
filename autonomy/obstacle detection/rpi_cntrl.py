import RPi.GPIO as GPIO
import time, sys, redis
import numpy as np
from dronestorm import DroneComm

host = '127.0.0.1'
r = redis.StrictRedis(host=host)
drone = DroneComm()

MID_WIDTH = 0.0015

try:
    while(True):
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

except (KeyboardInterrupt, SystemExit):
    # Graceful Exit
    drone.exit()
