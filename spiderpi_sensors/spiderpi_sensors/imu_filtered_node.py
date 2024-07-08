import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
import math
from spiderpi_sensors.complementary_filter import ComplementaryFilter
from spiderpi_sensors.kalman_filter import KalmanFilter

class IMUFilterNode(Node):
    def __init__(self):
        super().__init__('imu_filter_node')
        self.publisher = self.create_publisher(Imu, '/filtered_imu', 10)
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

    def imu_callback(self, msg):
        # Extract raw accelerometer and gyroscope data from Imu message
        accel_data = {'x': msg.linear_acceleration.x, 'y': msg.linear_acceleration.y, 'z': msg.linear_acceleration.z}
        gyro_data = {'x': msg.angular_velocity.x, 'y': msg.angular_velocity.y, 'z': msg.angular_velocity.z}

        # Use complementary filter to estimate orientation
        dt = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.last_timestamp if self.last_timestamp else 0.0
        orientation_angles = self.complementary_filter.update(accel=accel_data, gyro=gyro_data, dt=dt)
        
        # Remove gravity components from measurement
        gravity_x = self.gravity * math.sin(orientation_angles[1])
        gravity_y = -self.gravity * math.sin(orientation_angles[0])
        gravity_z = self.gravity * math.cos(orientation_angles[0]) * math.cos(orientation_angles[1])
        
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
                velocity = self.calculate_velocity(accel_data)
        else:
            velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
        self.get_logger().info(f'Estimated velocity is vx : {velocity['x']}, vy : {velocity['y']}, vz : {velocity['z']}')
        
        # Update last timestamp
        self.last_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Populate Imu message
        filtered_imu_msg = Imu()
        filtered_imu_msg.header = Header()
        filtered_imu_msg.header.stamp = self.get_clock().now().to_msg()
        filtered_imu_msg.header.frame_id = 'imu_link'

        filtered_imu_msg.orientation = quaternion

        filtered_imu_msg.linear_acceleration.x = accel_data['x']
        filtered_imu_msg.linear_acceleration.y = accel_data['y']
        filtered_imu_msg.linear_acceleration.z = accel_data['z']

        filtered_imu_msg.angular_velocity.x = gyro_data['x']
        filtered_imu_msg.angular_velocity.y = gyro_data['y']
        filtered_imu_msg.angular_velocity.z = gyro_data['z']
        
        # Publish filtered Imu message
        self.publisher.publish(filtered_imu_msg)

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

    def calculate_velocity(self, accel_data):
        # Calculate time difference
        current_timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        dt = current_timestamp - self.last_timestamp

        # Acceleration in global frame (assuming no rotation)
        accel_global = accel_data
        
        self.kalman_filter.predict(accel_data, dt)

        # Apply Kalman filter to estimate velocity
        velocity = self.kalman_filter.update(accel_global, self.last_acceleration, dt)

        # Update variables for next iteration
        self.last_timestamp = current_timestamp
        self.last_acceleration = accel_global

        return velocity

def main(args=None):
    rclpy.init(args=args)
    imu_filter_node = IMUFilterNode()
    rclpy.spin(imu_filter_node)
    imu_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
