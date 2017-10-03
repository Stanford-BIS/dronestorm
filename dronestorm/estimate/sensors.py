import numpy as np
from scipy.misc import derivative 
from state_matrix import * 
''' 
X(0): x (global)
X(1): y (global)
X(2): z (global)
X(3): phi
X(4): theta
X(5): psi
X(6): xdot (global)
X(7): ydot (global)
X(8): zdot (global)
X(9): wx
X(10): wy
X(11): wz
'''

def IMU(X, U, X0=None):
    '''
    Z[0] = xdd_body
    Z[1] = ydd_body
    Z[2] = zdd_body
    Z[3] = wx
    Z[4] = wy
    Z[5] = wz
    '''
    Z = np.zeros([6,])

    X_imu = np.array([0.01,0.01,0.01])

    R = rot(X[3],X[4],X[5])
    Xd = x_dot(X, U)

    rot_accel = np.cross(X_imu, X[9:12].squeeze())
    # import pdb; pdb.set_trace()
    Z[0:3] = (R.dot(Xd[6:9]).squeeze() + rot_accel)
    Z[3:6] = X[9:12].squeeze()
    return Z 

def AGL(X, U=None, X0=None):
    z = X[2]/(np.cos(X[3])*np.cos(X[4]))
    return z 

def GPS(X, U, X0):
    Z = np.zeros([6,])
    Z[0:3] = (X[0:3] + X0[0:3]).squeeze()
    Z[3] = np.linalg.norm(X[6:9])
    Z[4] = np.arctan2(X[7],X[6])
    Z[5] = X[8]
    return Z 

def gen_H(func, X, U, X0):
    # H = np.zeros([0,12])
    for i in range(12):
        Xhi = X[:]*1.0
        Xhi[i,0] = X[i,0] + 0.01
        Xlo = X[:]*1.0
        Xlo[i,0] = X[i,0] - 0.01

        Zhi = func(Xhi, U, X0)
        Zlo = func(Xlo, U, X0)
        # import pdb; pdb.set_trace()
        if 'H' in locals():
            H = np.hstack([H,(Zhi - Zlo)/(2.0*0.01)])
        else:
            H = (Zhi - Zlo)/(2.0*0.01) 
        return H 
