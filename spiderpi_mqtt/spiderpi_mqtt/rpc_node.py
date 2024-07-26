import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json

class RPCCommandPublisher(Node):
    def __init__(self):
        super().__init__('rpc_command_publisher')
        self.publisher_ = self.create_publisher(String, 'rpc_command', 10)
        self.timer = self.create_timer(1.0, self.publish_command)  # Timer callback every 1 second

    def send_rpc_request(self, method, params, rpc_id=9):
        url = "http://192.168.137.102:9030"
        headers = {"Content-Type": "application/json"}
        payload = {
            "method": method,
            "params": params,
            "jsonrpc": "2.0",
            "id": rpc_id
        }
        
        response = requests.post(url, headers=headers, data=json.dumps(payload))
        return response.json()

    def publish_command(self):
        response = self.send_rpc_request("Move", params, rpc_id=9)
        self.get_logger().info(f"RPC Response: {response}")

        # Publish the response as a ROS message
        message = json.dumps(response)
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RPCCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
