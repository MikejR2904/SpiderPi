import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3, Twist
import numpy as np
from queue import Queue, Empty
from threading import Thread, Lock
import time
from spiderpi_sensors.complementary_filter import ComplementaryFilter
from spiderpi_sensors.kalman_filter import KalmanFilter
from scipy.optimize import curve_fit

class IMUFilterNode(Node):
    def __init__(self):
        super().__init__('imu_filter_node')
        self.publisher = self.create_publisher(Twist, '/filtered_velocity', 10)
        self.orientation_pub = self.create_publisher(Vector3, '/estimated_orientation', 10)
        self.angular_velocity_pub = self.create_publisher(Vector3, '/estimated_angular_velocity', 10)
        self.linear_velocity_pub = self.create_publisher(Vector3, '/estimated_linear_velocity', 10)
        self.subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.subscription  # prevent unused variable warning

        # Initialize complementary filter for orientation estimation
        self.complementary_filter = ComplementaryFilter(alpha=0.99)

        # Initialize Kalman filter for velocity estimation
        self.kalman_filter = KalmanFilter()

        # Variables for velocity estimation
        self.last_timestamp = None
        self.last_acceleration = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.gravity = 9.80655
        
        # ZUPT parameters
        self.zupt_threshold = 0.1  # Adjust this threshold as per your application
        self.is_zupt = False
        
        # Gyro calibration offsets
        self.gyroXcal = 0.0
        self.gyroYcal = 0.0
        self.gyroZcal = 0.0
        
        # Accel calibration offsets
        self.accelXcal = 0.0
        self.accelYcal = 0.0
        self.accelZcal = 0.0
        
        self.count = 0
        
        # Queue for storing IMU data
        self.imu_data_queue = Queue()
        self.queue_lock = Lock()

        # Call the gyro calibration method
        Thread(target=self.calibrate_sensors, args=(100,)).start()

    def imu_callback(self, msg):
    	# Put IMU message into the queue
        with self.queue_lock:
            self.imu_data_queue.put(msg)
            
        if self.count < 100:
            return
    	
        # Extract raw accelerometer and gyroscope data from Imu message
        accel_data = {'x': msg.linear_acceleration.x - self.accelXcal, 'y': msg.linear_acceleration.y - self.accelYcal, 'z': msg.linear_acceleration.z - self.accelZcal}
        gyro_data = {'x': msg.angular_velocity.x - self.gyroXcal, 'y': msg.angular_velocity.y - self.gyroYcal, 'z': msg.angular_velocity.z - self.gyroZcal}

        # Use complementary filter to estimate orientation
        dt = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.last_timestamp if self.last_timestamp else 0.0
        orientation_angles = self.complementary_filter.update(accel=accel_data, gyro=gyro_data, dt=dt)
        
        # Convert degrees to radians
        roll = np.radians(orientation_angles[0])
        pitch = np.radians(orientation_angles[1])
        yaw = np.radians(orientation_angles[2])
        
        # Compute rotation matrices
        R_x = np.array([[1, 0, 0], [0, np.cos(roll), np.sin(roll)], [0, -np.sin(roll), np.cos(roll)]])
        R_y = np.array([[np.cos(pitch), np.sin(pitch), 0], [-np.sin(pitch), np.cos(pitch), 0], [0, 0, 1]])
        R_z = np.array([[np.cos(yaw), np.sin(yaw), 0], [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        
        # Combine rotation matrices to get XYZ Euler Angles
        R = np.dot(R_z, np.dot(R_y, R_x))
        g_global = np.array([0, 0, self.gravity])
        g_local = np.dot(R, g_global)
        
        # Remove gravity components from measurement
        gravity_x = g_local[0]
        gravity_y = g_local[1]
        gravity_z = g_local[2]
        
        corrected_acc_x = accel_data['x'] - gravity_x
        corrected_acc_y = accel_data['y'] - gravity_y
        corrected_acc_z = accel_data['z'] - gravity_z
        
        accel_data = {'x': corrected_acc_x, 'y': corrected_acc_y, 'z': corrected_acc_z}

        # Assign orientation from filter to quaternion message
        quaternion = self.angle_to_quaternion(orientation_angles)

        # Detect ZUPT (Zero Velocity Update)
        accel_norm = math.sqrt(accel_data['x'] ** 2 + accel_data['y'] ** 2 + accel_data['z'] ** 2)
        if accel_norm < self.zupt_threshold:
            self.is_zupt = True
        else:
            self.is_zupt = False

        # Estimate velocity using Kalman filter
        if self.last_timestamp is not None:
            if self.is_zupt:
                velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # Zero velocity during ZUPT
            else:
                velocity = self.calculate_velocity(accel_data, dt)
                self.kalman_filter.predict(acceleration=accel_data, dt=dt)
        else:
            velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
        self.get_logger().info(f'Estimated velocity is : {velocity}')
        
        # Update last timestamp
        self.last_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Populate Twist message
        twist_msg = Twist()
        twist_msg.linear.x = velocity['x']
        twist_msg.linear.y = velocity['y']
        twist_msg.linear.z = velocity['z']
        self.publisher.publish(twist_msg)
        
        orientation_msg = Vector3()
        orientation_msg.x = orientation_angles[0]
        orientation_msg.y = orientation_angles[1]
        orientation_msg.z = orientation_angles[2]
        self.orientation_pub.publish(orientation_msg)
        
        angular_velocity_msg = Vector3()
        angular_velocity_msg.x = gyro_data['x']
        angular_velocity_msg.y = gyro_data['y']
        angular_velocity_msg.z = gyro_data['z']
        self.angular_velocity_pub.publish(angular_velocity_msg)
        
        linear_velocity_msg = Vector3()
        linear_velocity_msg.x = velocity['x']
        linear_velocity_msg.y = velocity['y']
        linear_velocity_msg.z = velocity['z']
        self.linear_velocity_pub.publish(linear_velocity_msg)

    def angle_to_quaternion(self, angles):
        roll = angles[0]
        pitch = angles[1]
        yaw = angles[2]

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        return Quaternion(w=qw, x=qx, y=qy, z=qz)

    def calculate_velocity(self, accel_data, dt):
        # Acceleration in global frame (assuming no rotation)
        accel_global = accel_data
        
        self.kalman_filter.predict(accel_data, dt)
        
        # Apply Kalman filter to estimate velocity
        velocity = self.kalman_filter.update(accel_global)

        # Update variables for next iteration
        self.last_acceleration = accel_global

        return velocity
    
    def calibrate_sensors(self, N: int):
        # Display message
        self.get_logger().info("Starting sensor calibration. Keep the IMU steady.")
        
        # Calibrate gyroscope
        self.calibrate_gyroscope(N)
        
        # Calibrate accelerometer
        self.calibrate_accelerometer(N)

    def calibrate_gyroscope(self, N: int):
        # Display message
        self.get_logger().info("Calibrating gyro. Do not move!")
        
        while self.count < N:
            try:
                with self.queue_lock:
                    msg = self.imu_data_queue.get(timeout=1.0)  # Timeout in seconds
                    
                gyro_data = {'x': msg.angular_velocity.x, 'y': msg.angular_velocity.y, 'z': msg.angular_velocity.z}
                
                # Summing all the gyroscope values to be used for calibrating later
                self.gyroXcal += gyro_data['x']
                self.gyroYcal += gyro_data['y']
                self.gyroZcal += gyro_data['z']
                
                self.count += 1

            except Empty:
                self.get_logger().warn("IMU queue is empty, gyro calibration incomplete.")
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error("An error occured. Calibration stopped.")
                break
        
        # Averaging the number of points taken to get the calibration
        if self.count > 0:
            self.gyroXcal /= self.count
            self.gyroYcal /= self.count
            self.gyroZcal /= self.count
        
        # Inform that the calibration is done
        self.get_logger().info("Gyro calibration complete.")
        time.sleep(2)

    def calibrate_accelerometer(self, N: int):
        # Display message
        self.get_logger().info("Calibrating accelerometer. Do not move!")
        
        accel_x = []
        accel_y = []
        accel_z = []
        
        while len(accel_x) < N:
            try:
                with self.queue_lock:
                    msg = self.imu_data_queue.get(timeout=1.0)  # Timeout in seconds
                    
                accel_x.append(msg.linear_acceleration.x)
                accel_y.append(msg.linear_acceleration.y)
                accel_z.append(msg.linear_acceleration.z)
                
            except Empty:
                self.get_logger().warn("IMU queue is empty, accel calibration incomplete.")
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error("An error occured. Calibration stopped.")
                break
        
        # Convert lists to numpy arrays for easy manipulation
        accel_x = np.array(accel_x)
        accel_y = np.array(accel_y)
        accel_z = np.array(accel_z)
        
        # Fit a linear model to determine accelerometer biases (offsets)
        def linear_model(x, a, b):
            return a * x + b
        
        def fit_calibration_data(data):
            x_data = np.arange(len(data))
            popt, _ = curve_fit(linear_model, x_data, data)
            return popt[1]  # Return the intercept (offset)
        
        self.accelXcal = fit_calibration_data(accel_x)
        self.accelYcal = fit_calibration_data(accel_y)
        self.accelZcal = fit_calibration_data(accel_z)
        
        self.get_logger().info(f"Accelerometer calibration complete. Offsets: X={self.accelXcal}, Y={self.accelYcal}, Z={self.accelZcal}")  
        time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    imu_filter_node = IMUFilterNode()
    rclpy.spin(imu_filter_node)
    imu_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
