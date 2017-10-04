import numpy as np

def rot(phi, theta, psi):
	PHI = np.array([[1,0,0],
					[0,np.cos(phi),np.sin(phi)],
					[0,-np.sin(phi),np.cos(phi)]])
	THETA = np.array([[np.cos(theta),0,-np.sin(theta)],
					  [0,1,0],
					  [np.sin(theta),0,np.cos(theta)]])
	PSI = np.array([[np.cos(psi), np.sin(psi), 0],
					[-np.sin(psi), np.cos(psi), 0],
					[0,0,1]])
	R = PHI.dot(THETA.dot(PSI))
	return R 

def x_dot(X, U):
	R = rot(X[3], X[4], X[5]) # ZERO INDEXED...
	v_global = np.array([X[6],X[7],X[8]])
	v_body = R.dot(v_global)

	Iv = np.eye(3)*0.1
	mass = 0.75
	g = 9.81
	th_u = 3.975e-8
	th_v = -0.00882
	tr_u = 1.356e-9
	tr_v = -0.0002411
	torque_direction = [1.0,-1.0,-1.0,1.0]
	motor_position = np.array([[0.1,0.05,0],
							   [0.1,-0.05,0],
							   [-0.1,0.05,0],
							   [-0.1,-0.05,0]])

	Thrust = np.zeros([4,1])
	Torque = np.zeros([4,1])
	for i in range(4):
		Thrust[i] = -1.0*np.maximum((th_u*U[i]**2 + th_v*v_body[2]**2), 0)
		Torque[i] = torque_direction[i]*np.maximum((tr_u*U[i]**2 + tr_v*v_body[2]**2),0)
	# print(np.sum(Thrust))

	F_thrust_body = np.array([0,0,np.sum(Thrust)]).T
	F_thrust_global = R.T.dot(F_thrust_body)

	AoAx = np.rad2deg(X[3])
	AoAy = np.rad2deg(X[4])
	AoAz = 90 - np.rad2deg(np.abs(X[3]))

	Clx = 0.17*(AoAx + 45)**2/45**2 - 0.18
	Cdx = (0.6-0.25)*np.abs(AoAx)/90 + 0.25

	Cly = 0.17*(AoAy + 45)**2/45**2 - 0.18
	Cdy = (0.6-0.25)*np.abs(AoAy)/90 + 0.25

	Cdz = (0.6-0.25)*np.abs(AoAz)/90 + 0.25

	rho = 1.2250
	F_aero_global = np.zeros([3,])
	F_aero_global[0] = -0.5*rho*np.abs(X[6])*X[6]*Cdx 
	F_aero_global[1] = -0.5*rho*np.abs(X[7])*X[7]*Cdy
	F_aero_global[2] = 0.5*rho*(-Clx*np.abs(X[6])*X[6] - Cly*np.abs(X[7])*X[7] - Cdz*np.abs(X[8])*X[8])
	# print(F_aero_global)

	F_grav_global = np.zeros([3,])
	F_grav_global[2] = g*mass

	F_net_global = F_thrust_global + F_aero_global + F_grav_global
	A_global = F_net_global/mass

	M_body = np.zeros([3,])
	for i in range(4):
		M_body += np.cross(motor_position[i,:], np.array([0,0,Thrust[i]])) + np.array([0,0,Torque[i]])

	W_body = np.linalg.inv(Iv).dot(M_body.reshape([3,1]))

	Xd = np.zeros([12,1])
	Xd[0] = X[0]
	Xd[1] = X[7]
	Xd[2] = X[8]
	Xd[3] = X[9] + (X[10]*np.sin(X[3]) + X[11]*np.cos(X[3]))*np.tan(X[4])
	Xd[4] = X[10]*np.cos(X[3]) - X[11]*np.sin(X[3])
	Xd[5] = (X[10]*np.sin(X[3]) + X[11]*np.cos(X[3]))/np.cos(X[4])
	Xd[6:9] = A_global.reshape([3,1])
	Xd[9:12] = W_body.reshape([3,1])
	return Xd

def state_matrix(X0, U0, Xdot):
	X0 = X0.astype(float)
	U0 = U0.astype(float)
	A = np.zeros([12,12])
	for i in range(12):
		Xhi = X0[:]*1.0
		Xhi[i] = X0[i] + 0.01
		Xlo = X0[:]*1.0
		Xlo[i] = X0[i] - 0.01

		Xdhi = Xdot(Xhi, U0)
		Xdlo = Xdot(Xlo, U0)
		A[:,i] = ((Xdhi - Xdlo)/(2*0.01)).squeeze()
	B = np.zeros([12,4])
	for i in range(4):
		Uhi = U0[:]*1.0
		Uhi[i] = U0[i] + 10.0
		Ulo = U0[:]*1.0
		Ulo[i] = U0[i] - 10.0
		Xdhi = Xdot(X0, Uhi)
		Xdlo = Xdot(X0, Ulo)
		B[:,i] = ((Xdhi - Xdlo)/(2*10.0)).squeeze()
	C = np.zeros([4,12])
	C[0,6] = 1
	C[1,7] = 1
	C[2,8] = 1
	C[3,11] = 1
	D = np.zeros(4)
	return A, B, C, D

