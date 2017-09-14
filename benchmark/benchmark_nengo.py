# benchmark nengo simulator
import time
import nengo

net = nengo.Network()
with net:
    ens = nengo.Ensemble()

sim = nengo.Simulator(net)
