import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_geometry_msgs import PoseStamped
import math
from imu_filtered import IMUKalmanFilter

class ImuOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.broadcaster = TransformBroadcaster(self)
        self.kalman_filter = IMUKalmanFilter()

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
        odom_msg.twist.twist.linear.x = velocity[0, 0]  # Replace with your estimated linear velocity
        odom_msg.twist.twist.linear.y = velocity[1, 0]
        odom_msg.twist.twist.linear.z = velocity[2, 0]
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity[2]

        self.odom_pub.publish(odom_msg)

        # Broadcast TF transform (optional)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0  # Replace with your estimated position
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = orientation[0, 0]
        t.transform.rotation.y = orientation[1, 0]
        t.transform.rotation.z = orientation[2, 0]
        t.transform.rotation.w = 1.0  # Assuming no rotation around base_link's Z-axis
        self.broadcaster.sendTransform(t)

    # code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
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

        return roll_x, pitch_y, yaw_z # in radians

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
