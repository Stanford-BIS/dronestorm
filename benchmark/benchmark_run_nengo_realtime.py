# nengo wrapper
from __future__ import print_function
import time
import timeit
import nengo
import numpy as np
from dronestorm.nengo_util import run_nengo_realtime

SIM_TIME = 1

for nneurons in [20, 50, 100, 200, 500, 1000, 2000]:
    print("benchmarking %d neuron network..."%(nneurons))
    start = timeit.default_timer()
    net = nengo.Network()
    with net:
        stim = nengo.Node(lambda t: np.sin(t))
        ens_a = nengo.Ensemble(nneurons//2, 1)
        ens_b = nengo.Ensemble(nneurons//2, 1)
        out = nengo.Node(lambda t, x: x, size_in=1)
        nengo.Connection(stim, ens_a)
        nengo.Connection(stim, ens_b)
        nengo.Connection(ens_b, out, function=lambda x:x**2)
    sim = nengo.Simulator(net)
    run_nengo_realtime(sim, SIM_TIME, False)
    run_time = timeit.default_timer() - start
    print("ran %fs of simulation time in %fs of real time"%(
        SIM_TIME, run_time))
    print()
