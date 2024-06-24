import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import asyncio
import websockets
import json

class WebSocketClientNode(Node):

    def __init__(self):
        super().__init__('websocket_client')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'turtlebot_map', 10)  # Adjust topic name as needed
        self.uri = 'ws://turtlebot_ip:8765'  # Replace with TurtleBot3 IP address and WebSocket port

    async def connect_to_server(self):
        try:
            async with websockets.connect(self.uri) as websocket:
                self.get_logger().info(f"Connected to WebSocket server at {self.uri}")
                while True:
                    message = await websocket.recv()
                    self.get_logger().info(f"Received message: {message}")
                    
                    # Deserialize received JSON message to OccupancyGrid
                    try:
                        occupancy_grid = OccupancyGrid()
                        data_dict = json.loads(message)
                        occupancy_grid.header.frame_id = data_dict['header']['frame_id']
                        occupancy_grid.info.resolution = data_dict['info']['resolution']
                        occupancy_grid.info.width = data_dict['info']['width']
                        occupancy_grid.info.height = data_dict['info']['height']
                        occupancy_grid.info.origin.position.x = data_dict['info']['origin']['position']['x']
                        occupancy_grid.info.origin.position.y = data_dict['info']['origin']['position']['y']
                        occupancy_grid.info.origin.position.z = data_dict['info']['origin']['position']['z']
                        occupancy_grid.info.origin.orientation.x = data_dict['info']['origin']['orientation']['x']
                        occupancy_grid.info.origin.orientation.y = data_dict['info']['origin']['orientation']['y']
                        occupancy_grid.info.origin.orientation.z = data_dict['info']['origin']['orientation']['z']
                        occupancy_grid.info.origin.orientation.w = data_dict['info']['origin']['orientation']['w']
                        occupancy_grid.data = data_dict['data']
                        
                        # Publish OccupancyGrid message
                        self.publisher_.publish(occupancy_grid)
                        self.get_logger().info("Published OccupancyGrid message to ROS topic")

                    except Exception as e:
                        self.get_logger().error(f"Error deserializing message: {str(e)}")

        except Exception as e:
            self.get_logger().error(f"WebSocket connection error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    websocket_client = WebSocketClientNode()
    asyncio.get_event_loop().run_until_complete(websocket_client.connect_to_server())
    rclpy.spin(websocket_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
