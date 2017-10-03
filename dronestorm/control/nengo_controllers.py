"""Module for nengo control networks"""
import dronestorm.redis_util as redis_util
from dronestorm.nengo_util import RedisNodeGetRx, RedisNodeSetCmd, PrintNode
from dronestorm.print_util import print_control_none_data
import nengo

def print_wrapper(rx_cmd):
    """Wrapped print_control_none_data"""
    print_control_none_data(rx_cmd[:6], rx_cmd[6:])

def create_control_none_nengo(sim_dt=0.005, syn_tau=0.005):
    """Create a nengo network to foward receiver data to command data
    
    Parameters
    ----------
    sim_dt: float
        desired timestep of simulation (seconds)
    syn_tau: float
        synaptic time constant (seconds)
    """
    rdb = redis_util.DBRedis()
    net = nengo.Network()
    with net:
        stim = RedisNodeGetRx(rdb)
        ens = nengo.Ensemble(
            n_neurons=50, dimensions=6, neuron_type=nengo.Direct())
        preout = nengo.Node(lambda t, x: x, size_in=6, size_out=6)
        out = RedisNodeSetCmd(rdb)
        nengo.Connection(stim, ens, synapse=None)
        nengo.Connection(ens, preout, synapse=syn_tau)
        nengo.Connection(preout,out, synapse=None)
        # print signals
        print_node = PrintNode(print_wrapper, size_in=12)
        nengo.Connection(stim, print_node[:6], synapse=None)
        nengo.Connection(preout, print_node[6:], synapse=None)

    sim = nengo.Simulator(net, sim_dt, progress_bar=False)
    return sim
