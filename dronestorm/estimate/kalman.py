import numpy as np 
from state_matrix import * 
from sensors import *

class EKF: 
	def __init__(self, R):
		self.mu = np.zeros([12,1])
		self.sigma = np.eye(12)*0.01
		self.R = R
		self.sensor_idx = {0:GPS, 1:AGL, 2:IMU}
		self.Q_idx = {0: np.eye(6)*0.5,
					  1: np.eye(1)*0.01**2,
					  2: np.eye(9)*0.05**2
			}

	def initialize(self, Zgps):
		self.Z0 = {0:Zgps, 1:None, 2:None}

	def predict(self, U, dt):
		mud = x_dot(self.mu, U)
		mu_ = self.mu + mud*dt
		A, _, _, _ = state_matrix(self.mu, U, x_dot)
		sigma_ = (A.dot(self.sigma)).dot(A.T) + self.R
		return mu_, sigma_

	def update(self, mu_, sigma_, U, sensors=None, measurements=None):
		if sensors is not None: 
			for i, sensor in enumerate(sensors): 
				z_i = self.sensor_idx[sensor](mu_, U, self.Z0[sensor])
				H_i = gen_H(self.sensor_idx[sensor], mu_, U, self.Z0[sensor])
				zi = measurements[i]

				if 'z_' in locals():
					z_ = np.vstack([z_.reshape([-1,1]), z_i.reshape([-1,1])])
				else:
					z_ = z_i

				if 'z' in locals():
					z = np.vstack([z.reshape([-1,1]), zi.reshape([-1,1])])
				else:
					z = zi
				
				if 'H' in locals():
					H = np.vstack([H, H_i])
				else:
					H = H_i

				if 'Q' in locals():
					m, n = Q.shape
					Q_ = self.Q_idx[sensor]
					m_, n_ = Q_.shape
					Q = np.vstack([Q, np.zeros([m_,n])])
					Q__ = np.vstack([np.zeros([m, n_]), Q_])
					Q = np.hstack([Q, Q__])
				else:
					Q = self.Q_idx[sensor]
			y = z - z_ 
			S = H.dot(sigma_).dot(H.T) + Q 
			K = sigma_.dot(H.T).dot(np.linalg.pinv(S))
			self.mu = mu_ + (K.dot(y)).reshape([12,1])
			self.sigma = (np.eye(12) - K.dot(H)).dot(sigma_)
		else:
			self.mu = mu_
			self.sigma = sigma_
		self.mu = mu_
		self.sigma = sigma_ 

	def step(self, U, dt, sensors=None, measurements=None):
		mu_, sigma_ = self.predict(U, dt)
		self.update(mu_, sigma_, U, sensors, measurements)
		return self.mu, self.sigma 