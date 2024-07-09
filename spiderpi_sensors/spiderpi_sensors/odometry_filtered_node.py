import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_geometry_msgs import PoseStamped
import math
from spiderpi_sensors.imu_filtered_node import IMUFilterNode

class ImuOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.orientation_sub = self.create_subscription(Vector3, '/estimated_orientation', self.orientation_callback, 10)
        self.angular_velocity_sub = self.create_subscription(Vector3, '/estimated_angular_velocity', self.angular_velocity_callback, 10)
        self.linear_velocity_sub = self.create_subscription(Vector3, '/estimated_linear_velocity', self.linear_velocity_callback, 10)
        self.broadcaster = TransformBroadcaster(self)
        self.filter = IMUFilterNode()
        self.position = [0.0, 0.0, 0.0]  # Initial position in /map frame
        self.prev_time = None
        
        self.orientation = None
        self.angular_velocity = None
        self.linear_velocity = None

    def orientation_callback(self, msg):
    	self.orientation = msg
    
    def angular_velocity_callback(self, msg):
    	self.angular_velocity = msg
    	
    def linear_velocity_callback(self, msg):
    	self.linear_velocity = msg
	
    def imu_callback(self, msg):
        # Update Filter with new IMU data
        self.filter.imu_callback(msg)
        
        if self.orientation is None or self.angular_velocity is None or self.linear_velocity is None:
        	self.get_logger().info('Not all information is received')
        	return

        # Get estimated orientation and velocity from Kalman Filter
        orientation = self.orientation
        angular_velocity = self.angular_velocity
        velocity = self.linear_velocity

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'  # Adjust according to your robot's base frame
        # Set pose
        odom_msg.pose.pose.position.x = 0.0  # Replace with your estimated position
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.vector3_to_quaternion(orientation)
        # Set twist (if available)
        odom_msg.twist.twist.linear = velocity
        odom_msg.twist.twist.angular = angular_velocity

        self.odom_pub.publish(odom_msg)

        # Broadcast TF transform (optional)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        # Integrate velocity to update position in the TF transform
        if self.prev_time is None:
            self.prev_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            return

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        # Integrate linear velocity to get position
        t.transform.translation.x += velocity.x * dt
        t.transform.translation.y += velocity.y * dt
        t.transform.translation.z += velocity.z * dt

        self.broadcaster.sendTransform(t)
        
    def vector3_to_quaternion(self, vec3):
    	quaternion = Quaternion()
    	quaternion.x = vec3.x
    	quaternion.y = vec3.y
    	quaternion.z = vec3.z
    	quaternion.w = 1.0
    	return quaternion

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
