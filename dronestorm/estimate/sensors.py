import numpy as np 
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

def IMU(X, U, Z0=None):
    '''
    Z[0] = xdd_body
    Z[1] = ydd_body
    Z[2] = zdd_body
    Z[3] = wx
    Z[4] = wy
    Z[5] = wz
    '''
    Z = np.zeros([9,])

    X_imu = np.array([0.01,0.01,0.01])

    R_ = np.eye(3)
    R_[0,0] = -1
    R_[1,1] = -1
    R = rot(X[3],X[4],X[5])

    Xd = x_dot(X, U)
    rot_accel = np.cross(X_imu, X[9:12].squeeze())
    
    Z[0:3] = (R_.dot(X[3:6])).squeeze()
    Z[3:6] =  (R_.dot(R.dot(Xd[6:9]).squeeze() + rot_accel)).squeeze()
    W = X[9:12].squeeze()
    Z[6] = (W[0] + (W[1]*np.sin(X[3]) + W[2]*np.cos(X[3]))*np.tan(X[4])).squeeze()
    Z[7] = (W[1]*np.cos(X[3]) - W[2]*np.sin(X[3])).squeeze()
    Z[8] = ((W[1]*np.sin(X[3]) + W[2]*np.cos(X[3]))/np.cos(X[4])).squeeze()
    return Z 

def AGL(X, U=None, Z0=None):
    z = X[2]/(np.cos(X[3])*np.cos(X[4]))
    return z 

def GPS(X, U, Z0):
    '''
    Z[0] = Latitude
    Z[1] = Longitude
    Z[2] = Altitude
    Z[3] = Heading
    Z[4] = Speed
    Z[5] = Climb
    '''
    X = X.squeeze()
    Z = np.zeros([6,])
    Z[0] = Z0[0] + X[0]*np.cos(Z0[3]) - X[1]*np.sin(Z0[3])
    Z[1] = Z0[1] + X[1]*np.cos(Z0[3]) + X[0]*np.sin(Z0[3])
    Z[2] = Z0[2] + X[2]
    Z[3] = Z0[3] + X[5]
    Z[4] = np.linalg.norm(X[6:9])
    Z[5] = X[8]
    return Z 

def gen_H(func, X, U, X0):
    X = X.squeeze()
    for i in range(12):
        Xhi = X[:]*1.0
        Xhi[i] = X[i] + 0.01
        Xlo = X[:]*1.0
        Xlo[i] = X[i] - 0.01

        Zhi = func(Xhi, U, X0)
        Zlo = func(Xlo, U, X0)
        if 'H' in locals():
            H = np.hstack([H,((Zhi - Zlo)/(2.0*0.01)).reshape([-1,1])])
        else:
            H = ((Zhi - Zlo)/(2.0*0.01)).reshape([-1,1]) 
    return H 