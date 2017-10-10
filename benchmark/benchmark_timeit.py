"""Benchmark the Python timeit function

What is the smallest interval of time timeit can measure
"""
import time
import timeit
import numpy as np

def benchmark_timeit(n_trials):
    """collect timeit data"""
    dt_data = np.zeros(n_trials)
    for i in range(n_trials):
        start = timeit.default_timer()
        stop = timeit.default_timer()
        dt_data[i] = stop - start
    return dt_data

def print_stats(dt_data):
    """print statistics on the collected data"""
    mean_dt = np.mean(dt_data)
    std_dt = np.std(dt_data)
    median_dt = np.median(dt_data)
    min_dt = np.min(dt_data)
    max_dt = np.max(dt_data)
    print("mean:%.12f std:%.12f std/mean:%.12f median:%.12f min:%.12f max:%.12f"%(
        mean_dt, std_dt, std_dt/mean_dt, median_dt, min_dt, max_dt))

if __name__ == "__main__":
    print_stats(benchmark_timeit(n_trials=1000))
