"""Benchmark the redis_util"""
from __future__ import print_function
import time
import numpy as np
from dronestorm import redis_util

N = 100
rdb = redis_util.DBRedis()

# reset
start = time.time()
rdb.reset_db()
reset_dt = time.time() - start
print("Resetting the redis database took %.3fms"%(reset_dt*1000))

# set IMU data using set calls
set_dt = np.zeros(N)
for i in range(N):
    start = time.time()
    rdb.rdb.set(redis_util.REDIS_IMU_DROLL, 1000)
    rdb.rdb.set(redis_util.REDIS_IMU_DPITCH, 1000)
    rdb.rdb.set(redis_util.REDIS_IMU_DYAW, 1000)
    rdb.rdb.set(redis_util.REDIS_IMU_DDX, 1000)
    rdb.rdb.set(redis_util.REDIS_IMU_DDY, 1000)
    rdb.rdb.set(redis_util.REDIS_IMU_DDZ, 1000)
    set_dt[i] = time.time() - start
set_dt *= 1000 # to ms
print("setting imu data time mean:%.3fms median:%.3fms std:%.3fms"%(
    np.mean(set_dt), np.median(set_dt), np.std(set_dt)))

# get IMU data using get calls
get_dt = np.zeros(N)
for i in range(N):
    start = time.time()
    imu_droll = rdb.rdb.get(redis_util.REDIS_IMU_DROLL)
    imu_dpitch = rdb.rdb.get(redis_util.REDIS_IMU_DPITCH)
    imu_dyaw = rdb.rdb.get(redis_util.REDIS_IMU_DYAW)
    imu_ddx = rdb.rdb.get(redis_util.REDIS_IMU_DDX)
    imu_ddy = rdb.rdb.get(redis_util.REDIS_IMU_DDY)
    imu_ddz = rdb.rdb.get(redis_util.REDIS_IMU_DDZ)
    get_dt[i] = time.time() - start
get_dt *= 1000 # to ms
print("getting imu data time mean:%.3fms median:%.3fms std:%.3fms"%(
    np.mean(get_dt), np.median(get_dt), np.std(get_dt)))

# set IMU data using set into pipeline calls
pipelined_set_dt = np.zeros(N)
for i in range(N):
    start = time.time()
    redis_util.set_imu(rdb, [100, 100, 100, 100, 100, 100])
    pipelined_set_dt[i] = time.time() - start
pipelined_set_dt *= 1000 # to ms
print("pipelined setting imu data time mean:%.3fms median:%.3fms std:%.3fms"%(
    np.mean(pipelined_set_dt), np.median(pipelined_set_dt), np.std(pipelined_set_dt)))

# get IMU data using set/get into pipeline calls
pipelined_get_dt = np.zeros(N)
for i in range(N):
    start = time.time()
    imu_droll, imu_dpitch, imu_dyaw, imu_ddx, imu_ddy, imu_ddz = (
        redis_util.get_imu(rdb))
    pipelined_get_dt[i] = time.time() - start
pipelined_get_dt *= 1000 # to ms
print("pipelined getting imu data time mean:%.3fms median:%.3fms std:%.3fms"%(
    np.mean(pipelined_get_dt), np.median(pipelined_get_dt), np.std(pipelined_get_dt)))
