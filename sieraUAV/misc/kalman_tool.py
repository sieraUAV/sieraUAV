#!/usr/bin/python
"""
@package This package contains the class to drive a kalman filter
"""


import numpy as np
from cv2 import cv
from test_kalman import *


"""
This class enable to drive a kalman filter (2D) with cvkalman
"""
class kalman2D:

    def __init__(self, x_init=0, y_init=0, alive=False, m_noise_cov=1e-3, p_noise_cov=1e-5, err_cv_post=0.1):

        self.Kman= cv.CreateKalman(4, 2, 0)

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

    """
    Enable to correct the kalman with to coordinates
    @param x: coordinate of the image
    @param y: coordinate of the image
    """
    def correct(self, x, y):
        rightPoints = cv.CreateMat(2, 1, cv.CV_32FC1)
        rightPoints[0,0]=x
        rightPoints[1,0]=y

        #Correct kalman
        x= cv.KalmanCorrect(self.Kman, rightPoints)

        # return position estimation
        return (int(x[0,0]),int(x[1,0]))

    """
    Enable to correct the kalman with to coordinates
    @return the predition of the kalman filter (x and y coordinate)
    """
    def predict(self):
        x=cv.KalmanPredict(self.Kman)

        # return position prediction
        return (int(x[0,0]),int(x[1,0]))

    """
    Enable to init or reinnit the kalman filter
    """
    def init(self, x_init, y_init, alive=True, m_noise_cov=1e-3, p_noise_cov=1e-5, err_cv_post=0.1):
        self.__init__(x_init, y_init, alive, m_noise_cov, p_noise_cov, err_cv_post)


