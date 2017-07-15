import nengo
import redis
import numpy as np

#r = redis.StrictRedis(host='localhost')

def visualizer(t, x):
    x1 = 50+50*np.sin(x[0])
    y1 = 50-50*np.cos(x[0])

    visualizer._nengo_html_ = '''
        <svg width="100%" height="100%" viewbox="0 0 100 100">
            <line x1="{x1}" y1="{y1}" x2="50" y2="50" style="stroke:red; stroke-width:4"/>
        </svg>
        '''.format(**locals())
    #r.set('x',x1)
    #r.set('y',y1)        

model = nengo.Network()
with model:
    slider = nengo.Node([0])
    visual = nengo.Node(visualizer, size_in=1)
    nengo.Connection(slider, visual)
    