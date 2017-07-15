import nengo
import redis
import numpy as np
from xbox_controller import XBox


xbox = XBox()
xbox.start()

r = redis.StrictRedis(host='192.168.0.172')

def xsetter(t, x):
    r.set('rx', x[0])
def ysetter(t, y):
    r.set('ry', y[0])
def zsetter(t, z):
    r.set('rz', z[0])

model = nengo.Network()
with model:
    xslider = nengo.Node(lambda t:xbox.value['X1'])
    yslider = nengo.Node([0])
    zslider = nengo.Node([0])
    
    setx = nengo.Node(xsetter, size_in=1)
    sety = nengo.Node(ysetter, size_in=1)
    setz = nengo.Node(zsetter, size_in=1)
    
    nengo.Connection(xslider, setx)
    nengo.Connection(yslider, sety)
    nengo.Connection(zslider, setz)
    