from DroneControl import DroneComm, PID
import sys
import time
import redis
import numpy as np

roll_pwm_trim = 10
pitch_pwm_trim = 10
yaw_pwm_trim = -10

drone = DroneComm(
    port=None,
    roll_pwm_trim=roll_pwm_trim,
    pitch_pwm_trim=pitch_pwm_trim,
    yaw_pwm_trim=yaw_pwm_trim,
    )

output_roll=0.1
output_pitch=0.2
output_yaw=0.3

try:
    drone.set_roll_rate(output_roll)
    drone.set_pitch_rate(output_pitch)
    drone.set_yaw_rate(output_yaw)
except (KeyboardInterrupt, SystemExit):
    drone.exit()
