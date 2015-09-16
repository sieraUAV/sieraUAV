#!/usr/bin/python
"""
@package This package contains the class to drive a kalman filter
"""


import numpy as np
import time
from cv2 import cv


"""
This class enable to drive a kalman filter (2D) with cvkalman
"""
class kalman2D:

	def __init__(self, x_init=0, y_init=0, alive=True, m_noise_cov=1e-3, p_noise_cov=1e-3, err_cv_post=0.5, dst_des=50, pos_err_des=(0.40,0.50), t_des=0.3 ):

		self.Kman= cv.CreateKalman(4, 2, 0)

		#bool state kalman
		self.track=False
		self.alive=alive

		# This happens only one time to initialize the kalman Filter with the first (x,y) point
		self.Kman.state_pre[0,0]  = x_init
		self.Kman.state_pre[1,0]  = y_init
		self.Kman.state_pre[2,0]  = 0
		self.Kman.state_pre[3,0]  = 0

		# set kalman transition matrix
		self.Kman.transition_matrix[0,0] = 1
		self.Kman.transition_matrix[1,1] = 1
		self.Kman.transition_matrix[2,2] = 1
		self.Kman.transition_matrix[3,3] = 1
		self.Kman.transition_matrix[0,2] = 1
		self.Kman.transition_matrix[1,3] = 1

		# set Kalman Filter
		cv.SetIdentity(self.Kman.measurement_matrix, cv.RealScalar(1))
		cv.SetIdentity(self.Kman.measurement_noise_cov, cv.RealScalar(m_noise_cov))
		cv.SetIdentity(self.Kman.process_noise_cov, cv.RealScalar(p_noise_cov))
		cv.SetIdentity(self.Kman.error_cov_post, cv.RealScalar(err_cv_post))

		#info about error
		self.disterr=None
		self.pos_err_des=pos_err_des

		#Correction time
		self.t_corr=None
		self.t_des=t_des

		#Decision value between prediction and correction point
		self.dst_des=dst_des

		#init with a correction
		init_pt = cv.CreateMat(2, 1, cv.CV_32FC1)
		init_pt[0,0]=x_init
		init_pt[1,0]=y_init
		cv.KalmanPredict(self.Kman)
		cv.KalmanCorrect(self.Kman, init_pt)

	"""
	Enable to correct the kalman with to coordinates
	@param x: coordinate of the image
	@param y: coordinate of the image
	"""
	def correct(self, x, y):
		rightPoints = cv.CreateMat(2, 1, cv.CV_32FC1)
		rightPoints[0,0]=x
		rightPoints[1,0]=y

		#get a prediction
		(xp,yp)=self.predict()

		if self.track==True:
			#Compute distance between prediction and detection point
			dst=((xp-x)**2+(yp-y)**2)**0.5
			#If correction point is valid: Correction
			if dst<self.dst_des:
				#get time of corection
				self.t_corr=time.time()
				x= cv.KalmanCorrect(self.Kman, rightPoints)
				return (int(x[0,0]),int(x[1,0]),True)
			else:
				return (xp,yp, False)
		else:
			#get time of corection
			self.t_corr=time.time()
			#Correct kalman
			cv.KalmanPredict(self.Kman)
			x= cv.KalmanCorrect(self.Kman, rightPoints)
			# return position estimation
			return (int(x[0,0]),int(x[1,0]),True)


	"""
	Enable to correct the kalman with to coordinates
	@return the predition of the kalman filter (x and y coordinate)
	"""
	def predict(self):
		x=cv.KalmanPredict(self.Kman)
		err=self.Kman.error_cov_post

		xerr= (err[0,0] + err[0,1] + err[0,2] + err[0,3])
		yerr= (err[1,0] + err[1,1] + err[1,2] + err[1,3])

		self.disterr=200*(xerr**2+yerr**2)**(0.5)

		#Hysterisis to determine the states of kalman
		(des_L,desH)=self.pos_err_des
		if self.track==True:
			#Decision to kill kalman
			t_timeout_des=time.time()-self.t_corr
			if self.disterr>desH or t_timeout_des>self.t_des:
				self.track=False
				self.alive=False

		else:
			if self.disterr<des_L:
				self.track=True
			else:
				self.track=False

		# return position prediction
		return (int(x[0,0]),int(x[1,0]))

