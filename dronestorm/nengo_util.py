"""This module defines helper functions to run nengo networks"""
from __future__ import print_function
from __future__ import division
import time
import timeit
import nengo
from nengo.utils.ensemble import tuning_curves
import numpy as np
import dronestorm.comm.redis_util as redis_util

# minimum sleep accurately implemented by the OS and hardware
# determine using benchmark/benchmark_sleep.py
MIN_SLEEP = 0.005

ATTITUDE_SCALE = 180.
ATTITUDE_SCALE_INV = 1./ATTITUDE_SCALE

def _print_run_nengo_realtime_stats(
        dt_target, dt_measured, save_runtime_stats):
    """Print the runtime stats of a run_nengo_realtime simulation
    
    Inputs
    ------
    dt_target: float
        targeted simulation dt
    dt_measured: numpy array of floats
        measured simulation dt
    save_runtime_stats: string or None
        if not None, filename of location to save runtime stats
    """
    n_samples = len(dt_measured)
    dt_mean = np.mean(dt_measured)
    dt_median = np.median(dt_measured)
    dt_std = np.std(dt_measured)
    ddt_measured = dt_target - dt_measured
    n_fast = np.sum(ddt_measured > 0)
    n_slow = np.sum(ddt_measured < 0)

    report = (
        "target dt: %.1fms\n"%(1000*dt_target) +
        "measured dt (N=%d) "%(n_samples) +
        "mean:%.3fms median:%.3fms std:%f.3ms std/mean:%.1f mean_dt/target_dt:%.1f\n"%(
            1000*dt_mean, 1000*dt_median, 1000*dt_std, dt_std/dt_mean, dt_mean/dt_target) +
        "simulation was " +
        "fast on %d of %d (%.1f%%) steps and "%(n_fast, n_samples, 100*n_fast/n_samples) +
        "slow on %d of %d (%.1f%%) steps"%(n_slow, n_samples, 100*n_slow/n_samples))

    print(report)
    if save_runtime_stats:
        assert isinstance(save_runtime_stats, str)
        with open(save_runtime_stats, 'w') as fhandle:
            fhandle.write(report)

def run_nengo_realtime(
        nengo_sim,
        print_runtime_stats=True, save_runtime_stats=None,
        save_probe_data=None, save_tuning_curves=None):
    """Runs a nengo simulation in as close to real time as possible

    If a simulation step runs faster than real time, throttle
    the step taking until the simulation runs in close to real time

    If the simulation step runs slower than real time, let the simulation
    run as fast as possible, and report that simulation ran slower than
    real time

    Inputs
    ------
    nengo_sim: nengo.Simulator instance
        simulator to run
    print_runtime_stats: bool
        whether to print out the runtime stats after the simulation ends
    save_runtime_stats: string or None
        if not None, filename of location to save runtime stats
    save_probe_data: dict or None
        if not None, dictionary of {nengo probe object: filename to save to, ...}
    save_tuning_curves: {ensemble object: file, ...} or None
        if not None, dictionary of {nengo ensemble object: filename to save to, ...}
    """
    assert isinstance(nengo_sim, nengo.Simulator), (
        "must pass in a nengo.Simulator instance to run_nengo_realtime")
    if save_probe_data is not None:
        assert isinstance(save_probe_data, dict), (
            "save_probe_data must a dictionary mapping probe objects to filenames")
    if save_tuning_curves is not None:
        assert isinstance(save_tuning_curves, dict), (
            "save_tuning_curves must a dictionary mapping ensemble objects to filenames")

    dt_target = nengo_sim.dt
    dt_measured = []
    ddt_accumulated = 0 # accumulation of dt_measured - dt_target

    try:
        while True:
            start = timeit.default_timer()
            if ddt_accumulated >= MIN_SLEEP:
                time.sleep(ddt_accumulated)
                ddt_accumulated = 0
            nengo_sim.step()
            dt_measured.append(timeit.default_timer() - start)
            ddt_accumulated += dt_target - dt_measured[-1]
    except KeyboardInterrupt:
        print("\nkeyboard interrupt detected; ending simulation")

    if print_runtime_stats:
        _print_run_nengo_realtime_stats(dt_target, np.array(dt_measured), save_runtime_stats)

    if save_probe_data is not None:
        sim_time = nengo_sim.trange().reshape(-1, 1)
        measured_time = np.cumsum(dt_measured).reshape(-1, 1)
        len_sim_time = len(sim_time)
        len_measured_time = len(measured_time)
        for p_obj, fname in save_probe_data.items():
            probe_data = nengo_sim.data[p_obj]
            clip_idx = np.min((len_sim_time, len_measured_time, probe_data.shape[0]))
            np.savetxt(
                fname,
                np.hstack((sim_time[:clip_idx], measured_time[:clip_idx], probe_data[:clip_idx])),
                fmt="%.4f")

    if save_tuning_curves:
        for ens_obj, fname in save_tuning_curves.items():
            inputs = np.sort(
                np.linspace(-1, 1, 200).tolist()+nengo_sim.data[ens_obj].intercepts.tolist())
            inputs = inputs.reshape((-1, 1))
            tuning_data = tuning_curves(ens_obj, nengo_sim, inputs=inputs)
            np.savetxt(fname, np.hstack((tuning_data[0], tuning_data[1])), fmt="%.3f")

class RedisNodeGetAttitude(nengo.Node):
    """nengo Node for getting attitude data from redis"""
    def __init__(self, dbredis):
        self.dbredis = dbredis
        super(RedisNodeGetAttitude, self).__init__(
            self.update, size_in=0, size_out=3)

    def update(self, t):
        """update called with each step of the simulation"""
        raw_attitude = np.array(redis_util.get_attitude(self.dbredis))
        scaled_attitude = raw_attitude * ATTITUDE_SCALE_INV
        return scaled_attitude

class RedisNodeGet(nengo.Node):
    """nengo Node for getting attitude data from redis"""
    def __init__(self, dbredis, redis_key, value_fun=None, size_out=1):
        self.dbredis = dbredis
        self.redis_key = redis_key
        self.value_fun = value_fun
        super(RedisNodeGet, self).__init__(
            self.update, size_in=0, size_out=size_out)

    def update(self, t):
        """update called with each step of the simulation"""
        return np.array(redis_util.get_key(self.dbredis, self.redis_key, self.value_fun))

class RedisNodeGetRx(nengo.Node):
    """nengo Node for getting normalized receiver data from redis"""
    def __init__(self, dbredis):
        self.dbredis = dbredis
        super(RedisNodeGetRx, self).__init__(
            self.update, size_in=0, size_out=6)

    def update(self, t):
        """update called with each step of the simulation"""
        return np.array(redis_util.get_rx(self.dbredis))

class RedisNodeSetCmd(nengo.Node):
    """nengo Node for setting command data on redis"""
    def __init__(self, dbredis):
        self.dbredis = dbredis
        super(RedisNodeSetCmd, self).__init__(
            self.update, size_in=6, size_out=0)

    def update(self, t, x):
        """update called with each step of the simulation"""
        redis_util.set_cmd(self.dbredis, x)

class PrintNode(nengo.Node):
    """nengo Node for printing convienence"""
    def __init__(self, print_fun, size_in):
        self.print_fun = print_fun
        super(PrintNode, self).__init__(
            self.update, size_in=size_in, size_out=0)

    def update(self, t, x):
        """update called with each step of the simulation"""
        self.print_fun(x)
