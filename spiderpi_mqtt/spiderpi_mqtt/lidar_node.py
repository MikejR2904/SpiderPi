#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import paho.mqtt.client as mqtt
from std_msgs.msg import Header

import json

class LidarPublisherNode(Node):

    def __init__(self):
        super().__init__('lidar_publisher_node')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)  # Using sensor_msgs/Imu message type
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("192.168.137.102", 1883)  # Replace with your MQTT broker address and port
        self.get_logger().info("IMU Publisher Node initialized")
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker with result code {rc}")
        self.mqtt_client.subscribe("lidar/scan")
        self.get_logger().info("Subscribed to lidar/scan topic")

    def on_message(self, client, userdata, message):
    	payload = message.payload.decode()  # Convert payload bytes to string
    	data = json.loads(payload)
    	
    	scan_msg = LaserScan()
    	scan_msg.header = Header()
    	scan_msg.header.stamp = self.get_clock().now().to_msg()
    	scan_msg.header.frame_id = 'laser'
    	
    	scan_msg.angle_min = data['angle_min']
    	scan_msg.angle_max = data['angle_max']
    	scan_msg.angle_increment = data['angle_increment']
    	scan_msg.ranges = data['ranges']
    	scan_msg.intensities = data['intensities']
    	scan_msg.time_increment = data['time_increment']
    	scan_msg.scan_time = data['scan_time']
    	scan_msg.range_min = data['range_min']
    	scan_msg.range_max = data['range_max']
    	
    	self.publisher_.publish(scan_msg)
    	self.get_logger().info("Published successfully!")
        
    def destroy_node(self):
    	self.mqtt_client.loop_stop()
    	self.mqtt_client.disconnect()
    	super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
	main()
