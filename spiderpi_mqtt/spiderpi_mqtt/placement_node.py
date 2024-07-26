import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math

class OdometryUpdater(Node):
    def __init__(self):
        super().__init__('odometry_updater')
        self.subscription = self.create_subscription(
            String,
            'rpc_command',
            self.listener_callback,
            10)
        self.current_x = 0
        self.current_y = 0
        self.current_orientation = 0

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

    def handle_command(self, movement_direction, rotation, speed, times):
        distance = speed * times
        
        # Compute the new position
        new_x = self.current_x + distance * math.cos(math.radians(self.current_orientation + movement_direction))
        new_y = self.current_y + distance * math.sin(math.radians(self.current_orientation + movement_direction))
        
        # Update the position and orientation
        self.current_x = new_x
        self.current_y = new_y
        self.current_orientation = (self.current_orientation + rotation) % 360

        self.get_logger().info(f"Updated Position: ({self.current_x}, {self.current_y}), Updated Orientation: {self.current_orientation}")

    def listener_callback(self, msg):
        response = msg.data
        parsed_params = self.parse_response(response)
        if parsed_params:
            movement_direction, rotation, speed, times = parsed_params
            self.handle_command(movement_direction, rotation, speed, times)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
