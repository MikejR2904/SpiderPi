import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
from std_msgs.msg import String  # Import the message type you want to use for publishing

class WebSocketClient(Node):

    def __init__(self):
        super().__init__('websocket_client')
        self.uri = 'ws://mapping_robot_ip:8765'  # Replace with your mapping robot's IP and port
        self.publisher = self.create_publisher(String, 'mapping_data', queue_size=10)  # Create a publisher for String messages

    async def connect_to_server(self):
        async with websockets.connect(self.uri) as websocket:
            while True:
                mapping_data_json = await websocket.recv()
                mapping_data = json.loads(mapping_data_json)
                self.process_mapping_data(mapping_data)

    def process_mapping_data(self, mapping_data):
        # Publish mapping data to ROS topic
        msg = String()
        msg.data = json.dumps(mapping_data)  # Convert mapping_data to JSON string
        self.publisher.publish(msg)
        self.get_logger().info(f"Published mapping data: {msg.data}")

    def run(self):
        asyncio.get_event_loop().run_until_complete(self.connect_to_server())

def main(args=None):
    rclpy.init(args=args)
    client = WebSocketClient()
    client.run()
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
