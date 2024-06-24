import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Example message type, adjust as per your needs
import asyncio
import websockets

class WebSocketPublisher(Node):

    def __init__(self):
        super().__init__('websocket_publisher')
        self.subscription = self.create_subscription(
            String,  # Example message type, adjust as per your needs
            'publish_topic',  # Example topic name to subscribe to, adjust as per your needs
            self.listener_callback,
            10)
        self.uri = 'ws://turtlebot_ip:8765'  # Replace with the IP address and port of your WebSocket server

    def listener_callback(self, msg):
        try:
            asyncio.get_event_loop().run_until_complete(self.send_data_to_server(msg.data))
        except Exception as e:
            self.get_logger().error(f"Error sending data over WebSocket: {str(e)}")

    async def send_data_to_server(self, data):
        async with websockets.connect(self.uri) as websocket:
            await websocket.send(data)
            self.get_logger().info(f"Data sent over WebSocket: {data}")

def main(args=None):
    rclpy.init(args=args)
    websocket_publisher = WebSocketPublisher()
    rclpy.spin(websocket_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
