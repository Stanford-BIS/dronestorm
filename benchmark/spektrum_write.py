"""Tests the spektrum communication module"""
from __future__ import print_function
import time
import numpy as np
from dronestorm.comm import SpektrumRemoteReceiver

N = 100
MIN_WRITE_DT = 0.007 # adjust to find minimum time between writes
print("Benchmarking Spektrum Remote Receiver write...")
print("Please ensure that:")
print("  - the pc is connected properly to the flight control board")
print("Check that flight control board is getting RC data")
print("Ctrl-c when satisfied that flight control board is getting RC data")

dt_write = np.zeros(N)
rrx = SpektrumRemoteReceiver()
try:
    data = [1300 + i*100 for i in range(6)]
    idx = 0
    while True:
        start = time.time()
        rrx.write_data(data)
        time.sleep(MIN_WRITE_DT)
        dt_write[idx] = time.time() - start
        idx = (idx+1) % N
except(KeyboardInterrupt, SystemExit):
    rrx.close_serial()
except(Exception) as ex:
    print(ex)
    rrx.close_serial()

mean_dt_write = np.mean(dt_write)
median_dt_write = np.median(dt_write)
std_dt_write = np.std(dt_write)

print("write mean:%5.2fms median:%5.2fms std:%5.2fms std/mean:%5.2f"%(
    mean_dt_write*1000, median_dt_write*1000,
    std_dt_write*1000, std_dt_write/mean_dt_write))
