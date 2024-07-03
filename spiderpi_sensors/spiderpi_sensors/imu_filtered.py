import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.linalg import expm
import math

"More about Kalman Filter here : https://nitinjsanket.github.io/tutorials/attitudeest/kf"

class IMUKalmanFilter(Node):
    def __init__(self):
        super().__init__('imu_kalman_filter')

        # Initialize variables
        self.bias = np.zeros(3)  # Initialize bias vector for accelerometer
        self.is_stationary = False
        self.prev_time = None
        self.orientation = np.eye(3)  # Initial orientation matrix
        self.angular_velocity = np.zeros(3)  # Initial angular velocity
        self.velocity = np.zeros(3)  # Initial velocity (if estimating)

        # Kalman Filter initialization
        self.kf = KalmanFilter(dim_x=6, dim_z=6)
        self.kf.x = np.zeros(6)  # Initial state: [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
        self.kf.P *= 0.1  # Initial uncertainty
        self.kf.R = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Measurement noise
        self.kf.Q = np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])  # Process noise

        # Publishers and subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.twist_pub = self.create_publisher(Twist,'/filtered_velocity', 10)

    def imu_callback(self, msg):
        # Extract accelerometer and gyroscope data
        acc = np.array([msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])

        # Check if the robot is stationary
        self.detect_stationary(acc)

        # Apply ZUPT: Update accelerometer bias during stationary period
        if self.is_stationary:
            self.bias = 0.99 * self.bias + 0.01 * acc

            # Correct accelerometer readings
            acc_corrected = acc - self.bias
        else:
            acc_corrected = acc

        # Time integration using gyroscope readings (complementary filter)
        if self.prev_time is None:
            self.prev_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            return

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Complementary Filter: Orientation estimation
        self.update_orientation(gyro, dt)

        # Kalman Filter: State estimation (orientation and optionally velocity)
        self.update_kalman_filter(gyro, acc_corrected, dt)

        # Convert quaternion to Euler angles for logging or further processing
        roll, pitch, yaw = self.euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        # Publish filtered velocity (optional, modify based on your needs)
        twist_msg = Twist()
        twist_msg.linear.x = self.velocity[0]
        twist_msg.linear.y = self.velocity[1]
        twist_msg.linear.z = self.velocity[2]
        self.twist_pub.publish(twist_msg)

        # Log for debugging
        self.get_logger().info(f"Euler Angles: Roll={roll}, Pitch={pitch}, Yaw={yaw}")
        self.get_logger().info(f"Filtered Velocity: {self.velocity}")

    def detect_stationary(self, acc) -> None:
        # Example: Simple threshold-based stationary detection based on acceleration magnitude
        acc_magnitude = np.linalg.norm(acc)
        if acc_magnitude < 0.1:  # Adjust threshold as per your application
            self.is_stationary = True
        else:
            self.is_stationary = False

    def update_orientation(self, gyro, dt) -> None:
        # Complementary filter for orientation estimation
        gyro_matrix = np.array([[0, -gyro[2], gyro[1]],
                                [gyro[2], 0, -gyro[0]],
                                [-gyro[1], gyro[0], 0]]) # Skew-symmetrix matrix
        gyro_matrix_exp = expm(-gyro_matrix * dt) # exponential map for integration
        self.orientation = np.dot(self.orientation, gyro_matrix_exp)

    def update_kalman_filter(self, gyro, acc_corrected, dt) -> None:
        # Kalman filter update (orientation and optionally velocity)
        # State transition matrix (A)
        self.kf.F = np.array([[1, 0, 0, dt, 0, 0],
                              [0, 1, 0, 0, dt, 0],
                              [0, 0, 1, 0, 0, dt],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])

        # Measurement matrix (H)
        self.kf.H = np.eye(6)

        # Process noise covariance (Q)
        self.kf.Q = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])

        # Predict step
        self.kf.predict(u=gyro)

        # Measurement update (using accelerometer for orientation estimation)
        z = np.concatenate([acc_corrected, gyro])
        self.kf.update(z)
        
        # Extract orientation and angular velocity estimates from Kalman filter
        self.orientation = self.kf.x[:3]
        self.angular_velocity = self.kf.x[3:6]
        
        # Estimate velocity using orientation and accelerometer
        self.velocity = np.dot(self.orientation.T, acc_corrected)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z