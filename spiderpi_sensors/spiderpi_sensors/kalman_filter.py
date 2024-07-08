import numpy as np
import math

class KalmanFilter():
    def __init__(self):
        self.X = np.zeros(3)
        self.P = np.eye(3) * 0.1
        self.Q = np.eye(3) * 0.1
        self.R = np.eye(3) * 0.1
        self.H = np.eye(3)
        
    def predict(self, acceleration, dt):
        A = np.eye(3)
        B = np.eye(3) * dt
        self.X = np.dot(A, self.X) + np.dot(B, acceleration)
        self.P = np.dot(A, np.dot(self.P, A.T)) + self.Q
        
    def update(self, measurement):
        IM = np.dot(self.H, self.X)
        IS = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(IS)))
        
        self.X = self.X + np.dot(K, (measurement - IM))
        self.P = self.P - np.dot(K, np.dot(IS, K.T))
        
    def get_velocity(self):
        return self.X
