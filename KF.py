import numpy as np

class KalmanFilter:
    def __init__(self,_A,_B,_H,_x,_P,_Q,_R):
        self.A = _A                      # State transition matrix.
        self.B = _B                      # Control matrix.
        self.H = _H                      # Observation matrix.
        self.x = _x                      # Initial state estimate.
        self.P = _P                      # Initial covariance estimate.
        self.Q = _Q                      # Estimated error in process.
        self.R = _R                      # Estimated error in measurements (cov equipament).
    
    def SetA(self,_A):
        self.A = _A
    def SetR(self,_R):
        self.R = _R
    def state(self):
        return self.x.tolist()    
    def predict(self,_u):
        xEstimate = self.A * self.x + self.B * _u
        PEstimate = (self.A * self.P) * np.transpose(self.A) + self.Q
        return xEstimate,PEstimate
    def observe(self,_z,_xEstimate,_PEstimate):
        I = _z - self.H*_xEstimate
        Iv = self.H*_PEstimate*np.transpose(self.H) + self.R
        return I,Iv
    def update(self,_z,_xEstimate,_PEstimate,_I,_Iv):
        K = _PEstimate * np.transpose(self.H) * np.linalg.inv(_Iv)
        self.x = _xEstimate + K * _I
        size = self.x.shape[0]
        self.P = (np.eye(size) - K * self.H) * _PEstimate
    def iterate(self,_u,_z):
        xEstimate,PEstimate = self.predict(_u)
        I,Iv = self.observe(_z, xEstimate, PEstimate)
        self.update(_z, xEstimate, PEstimate, I, Iv)
