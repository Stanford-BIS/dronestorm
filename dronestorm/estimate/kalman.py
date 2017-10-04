"""Defined Kalman filters for state estimation"""
import numpy as np 
from state_matrix import * 
from sensors import *

class EKF: 
    """Extended Kalman filter
    

    dmu/dt = 

    Parameters
    ----------
    R: 
        noise covariance matrix. estimate of white noise

    Attributes
    ----------
    mu: 12 entry numpy column vector
        current state estimate
        [x, y, z, roll, pitch, yaw, dx, dy, dz, w_roll, w_pitch, w_yaw]
    sigma:
        current belief covariance 
    R
        plant noise covariance
    Q:
        sensor noise covariance
    Q_idx
    """
    def __init__(self, R):
        self.mu = np.zeros([12, 1])
        self.sigma = np.eye(12)*0.01
        self.R = R
        self.sensor_idx = {0:GPS, 1:AGL, 2:IMU}
        self.Q_idx = {0: np.eye(6)*4.5/2,
                      1: np.eye(1)*0.01,
                      2: np.eye(6)*0.05,}
    
    def initialize(self, Zgps, Zimu, Zagl):
        self.Z0 = {0:Zgps, 1:None, 2:None}
        # self.Zimu0 = Zimu
        # self.Zagl0 = Zagl
    
    def predict(self, U, dt):
        mud = x_dot(self.mu, U) # dmu/dt
        mu_ = self.mu + mud*dt
        # linarize around current mu for predicted sigma
        A, _, _, _ = state_matrix(self.mu, U, mud)
        sigma_ = (A.dot(self.sigma)).dot(A.T) + self.R
        return mu_, sigma_
    
    def update(self, mu_, sigma_, U, sensors=None, measurements=None):
        """Correction step"""
        if sensors is not None: 
            for i, sensor in enumerate(sensors): 
                z_i = self.sensor_idx[i](mu_, U, self.Z0[i])
                H_i = gen_H(self.sensor_idx[i], mu_, U, self.Z0[i])
                zi = measurements[i]
    
                if 'z_' in local():
                    z_ = np.vstack([z_, z_i])
                else:
                    z_ = z_i
    
                if 'z' in local():
                    z = np.vstack([z, zi])
                else:
                    z = zi
                
                if 'H' in local():
                    H = np.vstack([H, H_i])
                else:
                    H = H_i
    
                if 'Q' in local():
                    m, n = Q.shape
                    Q_ = self.Q_idx[i]
                    m_, n_ = Q_.shape
                    Q = np.vstack([Q, np.zeros([m_,n])])
                    Q__ = np.vstack([np.zeros([m, n_]), Q_])
                    Q = np.hstack([Q, Q__])
                else:
                    Q = self.Q_idx[i]
                y = z - z_  # predicted - measured
                S = H.dot(sigma_).dot(H.T) + Q 
                K = sigma_.dot(H.T).dot(np.linalg.inv(S)) # optimal Kalman gain
                self.mu = mu_ + K.dot(y)
                self.sigma = (np.eye(12) - K.dot(H)).dot(sigma_)
        else:
            self.mu = mu_
            self.sigma = sigma_
    
    def step(self, U, dt, sensors=None, measurements=None):
        mu_, sigma_ = self.predict(U, dt)
        self.update(mu_, sigma_, sensors, measurements)
        return self.mu, self.sigma 
