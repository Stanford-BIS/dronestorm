"""Benchmark the Python list"""
import numpy as np
import time
import timeit

def _print_stats(N, dt):
    """Utility print function"""
    mean_dt = np.mean(dt)
    std_dt = np.std(dt)
    median_dt = np.median(dt)
    max_dt = np.max(dt)
    print(
        "%d appends, "%(N) +
        "mean_dt:%.6fms std_dt:%.6fms median_dt:%.6fms std_dt/mean_dt:%.1f"%(
            1000*mean_dt, 1000*std_dt, 1000*median_dt, std_dt/mean_dt) +
        "max_dt:%.3fms"%(1000*max_dt))

def benchmark_append(N, plot=True):
    """Benchmark the append operation"""
    print("benchmarking append")
    x = []
    dt = []
    for n in range(N):
        start = timeit.default_timer()
        x.append(n)
        stop = timeit.default_timer()
        dt.append(stop - start)
    _print_stats(N, dt)
    return dt

def benchmark_set(N, plot=True):
    """Benchmark the set operation. Preallocate space and set each index"""
    print("benchmarking set")
    x = [0 for i in range(N)]
    dt = []
    for n in range(N):
        start = timeit.default_timer()
        x[n] = n
        stop = timeit.default_timer()
        dt.append(stop - start)
    _print_stats(N, dt)
    return dt

if __name__ == "__main__":
    plot = True
    append_dt = benchmark_append(1000000)
    set_dt = benchmark_set(1000000)
    
    if plot:
        import matplotlib.pyplot as plt
        plt.plot(append_dt)
        plt.plot(set_dt)
        plt.show()
