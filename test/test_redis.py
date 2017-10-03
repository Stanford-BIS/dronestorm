"""Test for redis_util"""
from __future__ import print_function
from dronestorm.comm import redis_util as r

rdb = r.DBRedis()
rdb.reset_db()
print(rdb.get_int(r.REDIS_IMU_DROLL))
rdb.set(r.REDIS_IMU_DROLL, 2000)
print(rdb.get_int(r.REDIS_IMU_DROLL))
