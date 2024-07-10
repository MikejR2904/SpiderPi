import numpy as np

class ComplementaryFilter():
    def __init__(self, alpha : float):
        self.alpha = alpha
        self.angle = np.zeros(3)
        self.last_time = None
        
    def update(self, accel : dict, gyro : dict, dt : float) -> np.ndarray:
        accel_angle = np.array([np.arctan2(accel['y'], np.sqrt(accel['x']**2+accel['z']**2)), np.arctan2(-accel['x'], np.sqrt(accel['y']**2 + accel['z']**2)), 0.0])
        
        if self.last_time is None:
            self.angle[:] = accel_angle
        else:
            gyro_angle = self.angle + np.array([gyro['x'], gyro['y'], gyro['z']]) * dt
            self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
            
        self.last_time = dt
        
        return self.angle
    
