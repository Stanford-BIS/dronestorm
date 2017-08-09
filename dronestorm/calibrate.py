"""Contains calibration routines"""
import time
import numpy as np

def calibrate_april_imu_yaw(redis, drone):
    redis.set('yaw', 0)
    while(redis.get('yaw') == '0'):
        print('waiting for camera to acquire april tag...')
        time.sleep(0.3)
    yaw_calib = float(redis.get('yaw'))*180./np.pi - drone.get_yaw()
    print("yaw calibration terimated with value %f"%yaw_calib)
    return yaw_calib

