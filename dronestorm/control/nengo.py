"""Module for nengo control networks"""
import dronestorm.redis_util as redis_util
from dronestorm.nengo_util import RedisNodeGetRx, RedisNodeSetCmd, PrintNode
from dronestorm.print_util import print_control_none_data
import nengo

def print_wrapper(rx_cmd):
    """Wrapped print_control_none_data"""
    print_control_none_data(rx_cmd[:6], rx_cmd[6:])

def create_control_none_nengo():
    """Create a nengo network to foward receiver data to command data"""
    tau_syn = 0.1
    rdb = redis_util.DBRedis()
    net = nengo.Network()
    with net:
        stim = RedisNodeGetRx(rdb)
        ens = nengo.Ensemble(
            n_neurons=50, dimensions=6, neuron_type=nengo.Direct())
        preout = nengo.Node(lambda t, x: x, size_in=6, size_out=6)
        out = RedisNodeSetCmd(rdb)
        nengo.Connection(stim, ens, synapse=None)
        nengo.Connection(ens, preout, synapse=tau_syn)
        nengo.Connection(preout,out, synapse=None)
        # print signals
        print_node = PrintNode(print_wrapper, size_in=12)
        nengo.Connection(stim, print_node[:6], synapse=None)
        nengo.Connection(preout, print_node[6:], synapse=None)

    sim = nengo.Simulator(net)
    return sim
