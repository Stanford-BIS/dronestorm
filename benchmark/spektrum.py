# module for benchmarking spektrum calls
# benchmark the Spektrum protocol communication
from __future__ import print_function
import sys
import time
import numpy as np
from dronestorm.comm import SpektrumRemoteReceiver

N = 100
rrx = SpektrumRemoteReceiver()

print("Benchmarking Spektrum Remote Receiver read...")
print("Please ensure the transmitter is on.")
dt_read = np.zeros(N)
rrx.align_serial()
for n in range(N):
    start = time.time()
    rrx.read_data()
    dt_read[n] = time.time()-start
mean_dt_read = np.mean(dt_read)
median_dt_read = np.median(dt_read)
std_dt_read = np.std(dt_read)

print("read mean:%5.2fms median:%5.2fms std:%5.2fms std/mean:%5.2f"%(
    mean_dt_read*1000, median_dt_read*1000,
    std_dt_read*1000, std_dt_read/mean_dt_read))
