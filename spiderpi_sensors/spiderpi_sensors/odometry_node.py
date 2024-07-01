import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_geometry_msgs import PoseStamped
import math

class ImuOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.broadcaster = TransformBroadcaster(self)

    def imu_callback(self, msg):
        # Process IMU data (e.g., orientation)
        orientation = msg.orientation
        (roll, pitch, yaw) = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        # Example: Simple integration for odometry
        # Update your pose estimation logic here based on IMU data
        # For example, integrate gyro for angular velocity and accelerometer for linear acceleration

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'  # Adjust according to your robot's base frame
        # Set pose
        odom_msg.pose.pose.position.x = 0.0  # Replace with your estimated position
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = orientation.x
        odom_msg.pose.pose.orientation.y = orientation.y
        odom_msg.pose.pose.orientation.z = orientation.z
        odom_msg.pose.pose.orientation.w = orientation.w
        # Set twist (if available)
        odom_msg.twist.twist.linear.x = 0.0  # Replace with your estimated linear velocity
        odom_msg.twist.twist.angular.z = 0.0  # Replace with your estimated angular velocity

        self.odom_pub.publish(odom_msg)

        # Broadcast TF transform (optional)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0  # Replace with your estimated position
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation
        self.broadcaster.sendTransform(t)

    def quaternion_to_euler(self, x, y, z, w):
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
