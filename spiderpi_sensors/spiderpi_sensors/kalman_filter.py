import numpy as np
import math

class KalmanFilter():
    def __init__(self):
        self.X = np.zeros((3,1))
        self.P = np.eye(3)
        self.Q = np.eye(3) * 0.01
        self.R = np.eye(3) * 0.1
        self.H = np.eye(3)
        
    def predict(self, acceleration, dt):
        A = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
        B = np.array([[0.5*dt**2], [dt], [1]])
        acceleration = np.array([[acceleration['x']], [acceleration['y']], [acceleration['z']]])
        self.X = np.dot(A, self.X) + np.dot(B, acceleration)
        self.P = np.dot(A, np.dot(self.P, A.T)) + self.Q
        
    def update(self, measurement):
    	if isinstance(measurement, dict):
    		measurement = np.array([[measurement['x'], [measurement['y'], measurement['z']])
        IM = np.dot(self.H, self.X)
        IS = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(IS)))
        
        self.X = self.X + np.dot(K, (measurement - IM))
        self.P = self.P - np.dot(K, np.dot(IS, K.T))
        
        return self.get_velocity()
        
    def get_velocity(self):
        return {'x': self.X[0][0], 'y': self.X[1][0], 'z': self.X[2][0]}
