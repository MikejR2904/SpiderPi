import numpy as np

class ComplementaryFilter():
    def __init__(self, alpha : float):
        self.alpha = alpha
        self.angle = np.zeros(3)
        self.last_time = None
        
    def update(self, accel : dict, gyro : dict, dt : float) -> np.ndarray:
        accel_angle = np.array([np.arctan2(accel['y'], accel['z']), np.arctan2(-accel['x'], np.sqrt(accel['x']**2 + accel['z']**2))])
        
        if self.last_time is None:
            self.angle[:2] = accel_angle
        else:
            gyro_angle = self.angle[:2] + np.array([gyro['x'], gyro['y']]) * dt
            self.angle[:2] = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
            
        self.last_time = dt
        
        return self.angle
    