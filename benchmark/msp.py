# benchmark the MultiWii Serial Protocol communication
import dronestorm.comm as comm
from dronestorm.comm import MultiWii, msp
from dronestorm.comm.msp.msp_types import RX_MSP, FIRMWARE_BF
import time
from pprint import pprint
import numpy as np

N = 100
mw = MultiWii(
    port="/dev/ttyACM0",
    rx_protocol=RX_MSP,
    firmware=FIRMWARE_BF)

print("benchmarking set motor command...")
dt_set_motor = np.zeros(N)
for n in range(N):
    start = time.time()
    msp.set_motor(mw, [1000 for i in range(4)])
    # print(msp.get_motor(mw))
    # print()
    dt_set_motor[n] = time.time() - start
mean_dt_set_motor = np.mean(dt_set_motor)
median_dt_set_motor = np.median(dt_set_motor)
std_dt_set_motor = np.std(dt_set_motor)

print("benchmarking set rc command...")
dt_set_raw_rc = np.zeros(N)
for n in range(N):
    start = time.time()
    msp.set_rc(mw, [1500 for i in range(6)])
    # print("checking that rc set to %d"%rc_value)
    # print(msp.get_rc(mw))
    # print()
    dt_set_raw_rc[n] = time.time() - start
mean_dt_set_raw_rc = np.mean(dt_set_raw_rc)
median_dt_set_raw_rc = np.median(dt_set_raw_rc)
std_dt_set_raw_rc = np.std(dt_set_raw_rc)
    
print("benchmarking get_raw_imu command...")
dt_get_raw_imu = np.zeros(N)
for n in range(N):
    start = time.time()
    msp.get_raw_imu(mw)
    dt_get_raw_imu[n] = time.time() - start
mean_dt_get_raw_imu = np.mean(dt_get_raw_imu)
median_dt_get_raw_imu = np.median(dt_get_raw_imu)
std_dt_get_raw_imu = np.std(dt_get_raw_imu)

mw.close_serial()

print("set_motor mean:%5.2fms median:%5.2fms std:%5.2fms std/mean:%5.2f"%(
    mean_dt_set_motor*1000, median_dt_set_motor*1000,
    std_dt_set_motor*1000, std_dt_set_motor/mean_dt_set_motor))
print("set_raw_rc mean:%5.2fms median:%5.2fms std:%5.2fms std/mean:%5.2f"%(
    mean_dt_set_raw_rc*1000, median_dt_set_raw_rc*1000,
    std_dt_set_raw_rc*1000, std_dt_set_raw_rc/mean_dt_set_raw_rc))
print("get_raw_imu mean:%5.2fms median:%5.2fms std:%5.2fms std/mean:%5.2f"%(
    mean_dt_get_raw_imu*1000, median_dt_get_raw_imu*1000,
    std_dt_get_raw_imu*1000, std_dt_get_raw_imu/mean_dt_get_raw_imu))
