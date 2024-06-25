import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import asyncio
import websockets
import json
import socket

class WebSocketNode(Node):
    
    TOPIC_MAP = {
        'turtlebot': {
            'map_publisher': '/turtlebot_map',
            'map_subscriber': '/spiderpi_map',
            'command_subscriber': '/spiderpi_command'
        },
        'spiderpi': {
            'map_publisher': '/spiderpi_map',
            'map_subscriber': '/turtlebot_map',
            'command_subscriber': '/turtlebot_command'
        }
    }

    def __init__(self, robot_type):
        self.robot_type = robot_type
        super().__init__(f'{self.robot_type}_websocket_node')
        self.uri = 'ws://common_websocket_server:8765'  # Replace with the IP address and port of your WebSocket server
        self.publisher = None
        self.subscription_map = None
        self.subscription_command = None

        self.setup_publisher_and_subscriptions()

    def setup_publisher_and_subscriptions(self):
        if self.robot_type in self.TOPIC_MAP:
            topic_map = self.TOPIC_MAP[self.robot_type]
            self.publisher = self.create_publisher(OccupancyGrid, topic_map['map_publisher'], 10)
            self.subscription_map = self.create_subscription(OccupancyGrid, topic_map['map_subscriber'], self.map_callback, 10)
            self.subscription_command = self.create_subscription(String, topic_map['command_subscriber'], self.command_callback, 10)
        else:
            self.get_logger().error("Unknown robot type. Cannot set up publisher and subscriptions.")
            return
        
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
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decoding error: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Error handling map data: {str(e)}")
            
    async def command_callback(self, msg):
        try:
            command_data = {
                'command': msg.data
            }
            await self.send_data_to_server(json.dumps(command_data))
            self.get_logger().info(f"{self.robot_type.capitalize()} command sent over WebSocket: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error handling {self.robot_type.capitalize()} command: {str(e)}")
        
    async def send_data_to_server(self, data):
        try:
            async with websockets.connect(self.uri) as websocket:
                await websocket.send(data)
                self.get_logger().info(f"Data sent over WebSocket: {data}")
        except websockets.exceptions.ConnectionClosed as e:
            self.get_logger().warning(f"WebSocket connection closed: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"WebSocket communication error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    # Determine robot type based on hostname
    hostname = socket.gethostname()
    if 'turtlebot' in hostname:
        robot_type = 'turtlebot'
    elif 'spiderpi' in hostname:
        robot_type = 'spiderpi'
    else:
        raise RuntimeError("Unknown hostname. Cannot determine robot type.")
    
    websocket_node = WebSocketNode(robot_type)
    asyncio.get_event_loop().run_until_complete(websocket_node.send_data_to_server(f'Hi from {robot_type}}!')) 
    rclpy.spin(websocket_node)  # Use rclpy.spin() if you have other nodes to spin
    rclpy.shutdown()

if __name__ == '__main__':
    main()
