"""Module for nengo control networks"""
import dronestorm.redis_util as redis_util
from dronestorm.nengo_util import RedisNodeGetRx, RedisNodeSetCmd

def create_nengo_control_none():
    """Create a nengo network to foward receiver data to command data"""
    rdb = redis_util.DBRedis()
    net = nengo.Network()
    with net:
        stim = RedisNodeGetRx(rdb)
        ens = nengo.Ensemble(
            n_neurons=50, dimensions=6, neuron_type=nengo.Direct())
        out = RedisNodeSetCmd(rdb)
        nengo.Connection(stim, ens, synapse=None)
        nengo.Connection(ens, out, synapse=0.01)
    sim = nengo.Simulator(net)
    return sim
