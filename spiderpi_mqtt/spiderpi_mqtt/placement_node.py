import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
import json
import math

class OdometryUpdater(Node):
    def __init__(self):
        super().__init__('odometry_updater')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.rpc_sub = self.create_subscription(String, 'rpc_command', self.rpc_callback, 10)
        self.broadcaster = TransformBroadcaster(self)
        self.current_x = 0
        self.current_y = 0
        self.current_orientation = 0
        self.prev_time = self.get_clock().now().to_msg().sec
        self.command = None
        self.command_start_time = None

    def parse_response(self, response):
        try:
            result = json.loads(response)
            if 'result' in result:
                params = result.get('result', {})
                movement_direction = params.get('movement_direction', 0)
                rotation = params.get('rotation', 0)
                speed = params.get('speed', 0)
                times = params.get('times', 0)
                return movement_direction, rotation, speed, times
            else:
                self.get_logger().info(f"Unexpected response format: {result}")
                return None
        except Exception as e:
            self.get_logger().error(f"Error parsing response: {e}")
            return None

    def update_odometry(self, movement_direction, rotation, speed, times):
        current_time = self.get_clock().now().to_msg().sec

        if self.command is None:
            self.command = (movement_direction, rotation, speed, times)
            self.command_start_time = current_time
            return

        if times == 0:
            # Continuous command
            dt = current_time - self.prev_time
            distance = speed * dt
        else:
            # Finite command
            dt = current_time - self.command_start_time
            if dt >= times:
                self.command = None
                self.prev_time = current_time
                return
            distance = speed * dt

        movement_direction, rotation, speed, _ = self.command

        # Update orientation due to rotation
        new_orientation = (self.current_orientation + rotation * (dt / times)) % 360

        if movement_direction != 0:
            # Update the position based on the movement direction and current orientation
            distance_x = distance * math.cos(math.radians(self.current_orientation + movement_direction))
            distance_y = distance * math.sin(math.radians(self.current_orientation + movement_direction))

            self.current_x += distance_x
            self.current_y += distance_y

        # Update the orientation
        self.current_orientation = new_orientation

        self.prev_time = current_time
        self.get_logger().info(f"Updated Position: ({self.current_x}, {self.current_y}), Orientation: {self.current_orientation}")

    def rpc_callback(self, msg):
        response = msg.data
        parsed_params = self.parse_response(response)
        if parsed_params:
            movement_direction, rotation, speed, times = parsed_params
            self.update_odometry(movement_direction, rotation, speed, times)
            self.publish_odometry()

    def publish_odometry(self):
        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.current_x
        odom_msg.pose.pose.position.y = self.current_y
        odom_msg.pose.pose.position.z = 0.0
        # Set orientation as a quaternion
        odom_msg.pose.pose.orientation = self.get_orientation_quaternion(self.current_orientation)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

        # Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.get_orientation_quaternion(self.current_orientation)

        self.broadcaster.sendTransform(t)

    def get_orientation_quaternion(self, orientation_deg):
        # Convert the orientation from degrees to a quaternion
        yaw = math.radians(orientation_deg)
        return TransformStamped().transform.rotation._replace(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2),
            w=math.cos(yaw / 2)
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdometryUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

