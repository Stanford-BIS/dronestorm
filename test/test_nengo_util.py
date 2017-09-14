# nengo wrapper
import time
import nengo
import numpy as np
from dronestorm.nengo_util import run_nengo_realtime

net = nengo.Network()
with net:
    stim = nengo.Node(lambda t: np.sin(t))
    ens = nengo.Ensemble(100, 1)
    out = nengo.Node(lambda t, x: x, size_in=1)
    nengo.Connection(stim, ens)
    nengo.Connection(ens, out, function=lambda x:x**2)
sim = nengo.Simulator(net)
run_nengo_realtime(sim, 1)
