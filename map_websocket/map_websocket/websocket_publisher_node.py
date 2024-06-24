import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import asyncio
import websockets
import json

class WebSocketPublisher(Node):

    def __init__(self):
        super().__init__('websocket_publisher')
        self.subscription = self.create_subscription(
            OccupancyGrid,  # Subscribe to OccupancyGrid messages
            '/map',  # Topic name where occupancy map data is published
            self.map_callback,
            10)
        self.uri = 'ws://turtlebot_ip:8765'  # Replace with the IP address and port of your WebSocket server

    async def map_callback(self, msg):
        try:
            # Convert OccupancyGrid message to JSON or any other suitable format
            map_data = {
                'header': {
                    'seq': msg.header.seq,
                    'stamp': {'secs': msg.header.stamp.sec, 'nsecs': msg.header.stamp.nanosec},
                    'frame_id': msg.header.frame_id
                },
                'info': {
                    'map_load_time': {'secs': msg.info.map_load_time.sec, 'nsecs': msg.info.map_load_time.nanosec},
                    'resolution': msg.info.resolution,
                    'width': msg.info.width,
                    'height': msg.info.height,
                    'origin': {
                        'position': {
                            'x': msg.info.origin.position.x,
                            'y': msg.info.origin.position.y,
                            'z': msg.info.origin.position.z
                        },
                        'orientation': {
                            'x': msg.info.origin.orientation.x,
                            'y': msg.info.origin.orientation.y,
                            'z': msg.info.origin.orientation.z,
                            'w': msg.info.origin.orientation.w
                        }
                    }
                },
                'data': msg.data  # OccupancyGrid data
            }

            # Send map_data over WebSocket
            await self.send_data_to_server(json.dumps(map_data))
            self.get_logger().info("Occupancy grid data sent over WebSocket")
        except Exception as e:
            self.get_logger().error(f"Error handling map data: {str(e)}")

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
