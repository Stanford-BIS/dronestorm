"""This module defines helper functions to run nengo networks"""
from __future__ import print_function
import time
import nengo
import numpy as np

def _print_run_nengo_realtime_stats(
        dt_target, dt_measured, dt_measured_full, idx):
    """Print the runtime stats of a run_nengo_realtime simulation"""
    if not dt_measured_full:
        n_samples = idx
        if idx == 0:
            dt_measured = 0
        else:
            dt_measured = dt_measured[:idx]
    else:
        n_samples = len(dt_measured)
    dt_mean = np.mean(dt_measured)
    dt_median = np.median(dt_measured)
    dt_std = np.std(dt_measured)

    print("target dt: %fs"%dt_target)
    print("measured dt (N=%d) mean:%fs median:%fs std:%fs std/mean:%f"%(
        n_samples, dt_mean, dt_median, dt_std, dt_std/dt_mean))

def run_nengo_realtime(
        nengo_sim, sim_stop_time=None, report_runtime_stats=True):
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

    sim_stop_time: None or float
        stop time of the simulator.
        if None, will run until interrupted, for example, with a ctrl-c
    
    report_runtime_stats : bool
        whether to print out the runtime stats after the simulation ends
    """
    assert isinstance(nengo_sim, nengo.Simulator), (
        "must pass in a nengo.Simulator instance to run_nengo_realtime")

    t_cur = 0
    t_stop = 0
    if sim_stop_time is not None:
        t_stop = sim_stop_time

    dt_target = nengo_sim.dt
    n_dt_samples = 100
    dt_measured = np.zeros(n_dt_samples)
    ddt = 0 # dt_measured - dt_target

    idx = 0
    dt_measured_full = False
    try:
        print("Running nengo simulation...")
        if sim_stop_time is None:
            print("Press Ctrl-c to end simulation")
        while t_cur < t_stop or sim_stop_time is None:
            start = time.time()

            if ddt < 0:
                ddt = 0
            time.sleep(ddt)

            nengo_sim.step()

            t_cur += dt_target
            dt_measured[idx] = time.time() - start
            ddt = dt_target - dt_measured[idx]

            idx += 1
            dt_measured_full = dt_measured_full | (idx == n_dt_samples)
            idx %= n_dt_samples
        print("nengo simulation finished")
    except KeyboardInterrupt:
        print("keyboard interrupt detected; ending simulation")

    _print_run_nengo_realtime_stats(
        dt_target, dt_measured, dt_measured_full, idx)
