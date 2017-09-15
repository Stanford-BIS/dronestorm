"""Benchmark the Python sleep fucntion

The accuracy of sleep() time depends on the underlying os.
Run this benchmark to determine the capacity of your system
"""
import time
import numpy as np

def time_sleep(amount):
    """time the runtime of sleep"""
    start = time.time()
    time.sleep(amount)
    end = time.time()
    delta = end-start
    return delta

def benchmark_sleep(min_sleep, max_sleep, n_trials):
    """collect sleep data"""
    log_min_sleep = np.log(min_sleep)
    log_max_sleep = np.log(max_sleep)
    tgt_sleep = np.exp(
        np.random.ranf(n_trials)*(log_max_sleep-log_min_sleep)+log_min_sleep)
    measured_sleep = np.zeros(n_trials)
    for idx in range(n_trials):
        measured_sleep[idx] = time_sleep(tgt_sleep[idx])
    return tgt_sleep, measured_sleep

def plot_data(tgt_sleep, measured_sleep):
    """plot the benchmark data"""
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.subplots(1, 1)
    min_sleep = np.min([tgt_sleep, measured_sleep])
    max_sleep = np.max([tgt_sleep, measured_sleep])
    ax.loglog(tgt_sleep, measured_sleep, 'o')
    ax.loglog([min_sleep, max_sleep], [min_sleep, max_sleep], 'r')
    ax.set_xlabel("target sleep time (s)")
    ax.set_ylabel("measured sleep time (s)")
    plt.show()

def run_benchmark(min_sleep=1E-6, max_sleep=1E-1, n_trials=100,
                  save=None, plot=True):
    """run the sleep benchmark"""
    tgt_sleep, measured_sleep = benchmark_sleep(min_sleep, max_sleep, n_trials)
    if save:
        save_dat = np.array((tgt_sleep, measured_sleep)).T
        np.savetxt(save, save_dat)
    if plot:
        plot_data(tgt_sleep, measured_sleep)

def load_and_plot_data(fname):
    """load data from fname and plot it"""
    data = np.loadtxt(fname)
    tgt_sleep = data[:, 0]
    measured_sleep = data[:, 1]
    plot_data(tgt_sleep, measured_sleep)

if __name__ == "__main__":
    run_benchmark()
    # run_benchmark(save="benchmark_sleep_data.txt")
    # load_and_plot_data("benchmark_sleep_data.txt")
