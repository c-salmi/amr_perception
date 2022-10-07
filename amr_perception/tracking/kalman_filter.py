import numpy as np


class KalmanFilter:
	def __init__(self, init_state, frequency, state_variance=1, measurement_variance=1, method="Velocity"):
		self.method = method
		self.stateVariance = state_variance
		self.measurementVariance = measurement_variance

		self.U = 1 if method == "Acceleration" else 0

		dt = 1/frequency
		self.A = np.matrix([
			[1, dt, 0, 0],
			[0, 1, 	0, 0],
			[0, 0,  1, dt],
			[0, 0,  0, 1]
		])

		self.B = np.matrix([
			[dt**2/2],
			[dt],
			[dt**2/2],
			[dt]
		])
		
		self.H = np.matrix([
			[1, 0, 0, 0],
			[0, 0, 1, 0]
		])

		self.P = np.matrix(self.stateVariance*np.identity(self.A.shape[0]))
		self.R = np.matrix(self.measurementVariance*np.identity(self.H.shape[0]))

		self.Q = np.matrix([
			[dt**4/4, dt**3/2, 0, 		0],
			[dt**3/2, dt**2,   0, 		0],
			[0, 	  0, 	   dt**4/4, dt**3/2],
			[0, 	  0, 	   dt**3/2, dt**2]
		])

		self.state = init_state
		self.pred_state = init_state

		self.err_cov = self.P
		self.pred_err_cov = self.P

	def predict(self):
		self.pred_state = self.A*self.state + self.B*self.U
		self.pred_err_cov = self.A*self.err_cov*self.A.T + self.Q

	def correct(self, measurement):
		kalman_gain = self.pred_err_cov*self.H.T*np.linalg.pinv(self.H*self.pred_err_cov*self.H.T+self.R)
		self.state = self.pred_state + kalman_gain*(measurement - (self.H*self.pred_state))
		self.err_cov = (np.identity(self.P.shape[0]) - kalman_gain*self.H)*self.pred_err_cov

