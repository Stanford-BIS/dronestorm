"""Benchmark the redis_util"""
from __future__ import print_function
import time
from dronestorm import redis_util as r

rdb = r.DBRedis()

# reset
start = time.time()
rdb.reset_db()
reset_dt = time.time() - start
print("Resetting the redis database took %.3fms"%(reset_dt*1000))

# set IMU data using set calls
start = time.time()
rdb.set(r.REDIS_IMU_DROLL, 1000)
rdb.set(r.REDIS_IMU_DPITCH, 1000)
rdb.set(r.REDIS_IMU_DYAW, 1000)
rdb.set(r.REDIS_IMU_DDX, 1000)
rdb.set(r.REDIS_IMU_DDY, 1000)
rdb.set(r.REDIS_IMU_DDZ, 1000)
set_dt = time.time() - start
print("setting imu data took %.3fms"%(set_dt*1000))

# get IMU data using get calls
start = time.time()
imu_droll = rdb.get_int(r.REDIS_IMU_DROLL)
imu_dpitch = rdb.get_int(r.REDIS_IMU_DPITCH)
imu_dyaw = rdb.get_int(r.REDIS_IMU_DYAW)
imu_ddx = rdb.get_int(r.REDIS_IMU_DDX)
imu_ddy = rdb.get_int(r.REDIS_IMU_DDY)
imu_ddz = rdb.get_int(r.REDIS_IMU_DDZ)
get_dt = time.time() - start
print("getting imu data took %.3fms"%(get_dt*1000))

# set IMU data using set into pipeline calls
start = time.time()
rdb.set_imu([100, 100, 100, 100, 100, 100])
pipelined_set_dt = time.time() - start
print("pipelined setting imu data took %.3fms"%(pipelined_set_dt*1000))

# get IMU data using set/get into pipeline calls
start = time.time()
imu_droll, imu_dpitch, imu_dyaw, imu_ddx, imu_ddy, imu_ddz = (
    rdb.get_imu())
pipelined_get_dt = time.time() - start
print("pipelined getting imu data took %.3fms"%(pipelined_get_dt*1000))
