import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_geometry_msgs import PoseStamped
import math
from spiderpi_sensors.imu_filtered import IMUKalmanFilter

class ImuOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.broadcaster = TransformBroadcaster(self)
        self.kalman_filter = IMUKalmanFilter()
        self.position = [0.0, 0.0, 0.0]  # Initial position in /map frame
        self.prev_time = None


    def imu_callback(self, msg):
        # Update Kalman Filter with new IMU data
        self.kalman_filter.imu_callback(msg)

        # Get estimated orientation and velocity from Kalman Filter
        orientation = self.kalman_filter.orientation
        angular_velocity = self.kalman_filter.angular_velocity
        velocity = self.kalman_filter.velocity

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'  # Adjust according to your robot's base frame
        # Set pose
        odom_msg.pose.pose.position.x = 0.0  # Replace with your estimated position
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = orientation[0, 0]
        odom_msg.pose.pose.orientation.y = orientation[1, 0]
        odom_msg.pose.pose.orientation.z = orientation[2, 0]
        odom_msg.pose.pose.orientation.w = 1.0  # Assuming no rotation around base_link's Z-axis
        # Set twist (if available)
        odom_msg.twist.twist.linear.x = velocity[0]  # Replace with your estimated linear velocity
        odom_msg.twist.twist.linear.y = velocity[1]
        odom_msg.twist.twist.linear.z = velocity[2]
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity[2]

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
        t.transform.translation.x += velocity[0] * dt
        t.transform.translation.y += velocity[1] * dt
        t.transform.translation.z += velocity[2] * dt

        # Set orientation in the TF transform
        t.transform.rotation.x = orientation[0, 0]
        t.transform.rotation.y = orientation[1, 0]
        t.transform.rotation.z = orientation[2, 0]
        t.transform.rotation.w = 1.0  # Assuming no rotation around base_link's Z-axis

        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
